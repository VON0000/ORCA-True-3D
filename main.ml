let eps = 1e-9

type runtime_agent = {
  id : string;
  goal : V3.t;
  radius : float;
  state : Avoid.state;
  reached : bool;
  stalled : bool;
  stall_steps : int;
}

type runtime_dynamic = {
  id : string;
  goal : V3.t;
  radius : float;
  speed : float;
  state : Avoid.state;
  reached : bool;
}

let pp_v3 (v : V3.t) = Printf.sprintf "(%.3f, %.3f, %.3f)" v.x v.y v.z
let stop_distance radius = max 0.15 (radius *. 0.75)
let bool_to_int b = if b then 1 else 0
let stall_window_seconds = 6.0

let stall_window_steps dt =
  max 6 (int_of_float (ceil (stall_window_seconds /. max eps dt)))

let stall_progress_threshold ~dt (params : Avoid.params) radius =
  max 1e-3 (max (0.01 *. params.max_speed *. dt) (0.005 *. radius))

let stall_clearance_threshold radius = max 1e-3 (0.01 *. radius)

let csv_float v =
  match classify_float v with
  | FP_nan | FP_infinite -> "nan"
  | _ -> Printf.sprintf "%.6f" v

let ensure_dir path =
  let rec mkdir_p dir =
    if dir = "" || dir = "." || dir = "/" then ()
    else if Sys.file_exists dir then ()
    else (
      mkdir_p (Filename.dirname dir);
      Unix.mkdir dir 0o755 )
  in
  let dir = Filename.dirname path in
  mkdir_p dir

let toward ~speed pos goal =
  let delta = V3.(goal - pos) in
  let dist = V3.norm delta in
  if dist < eps || speed <= 0.0 then V3.zero else V3.(speed /. dist * delta)

let make_runtime_agent (spec : Scene_config.agent) =
  let reached = V3.distance spec.start spec.goal <= stop_distance spec.radius in
  {
    id = spec.id;
    goal = spec.goal;
    radius = spec.radius;
    state =
      { Avoid.pos = (if reached then spec.goal else spec.start); vel = V3.zero };
    reached;
    stalled = false;
    stall_steps = 0;
  }

let make_runtime_dynamic (spec : Scene_config.moving_obstacle) =
  let reached = V3.distance spec.start spec.goal <= stop_distance spec.radius in
  let pos = if reached then spec.goal else spec.start in
  {
    id = spec.id;
    goal = spec.goal;
    radius = spec.radius;
    speed = spec.speed;
    state = { Avoid.pos; vel = toward ~speed:spec.speed pos spec.goal };
    reached;
  }

let obstacle_of_static (obs : Scene_config.static_obstacle) : Avoid.obstacle =
  { Avoid.pos = obs.pos; vel = V3.zero; radius = obs.radius; responsibility = 1.0 }

let obstacle_of_dynamic (obs : runtime_dynamic) : Avoid.obstacle =
  {
    Avoid.pos = obs.state.pos;
    vel = obs.state.vel;
    radius = obs.radius;
    responsibility = 1.0;
  }

let obstacle_of_agent (agent : runtime_agent) : Avoid.obstacle =
  {
    Avoid.pos = agent.state.pos;
    vel = agent.state.vel;
    radius = agent.radius;
    responsibility = 0.5;
  }

let clearance_to_obstacle pos radius (obs : Avoid.obstacle) =
  V3.distance pos obs.pos -. (radius +. obs.radius)

let min_clearance pos radius obstacles =
  match obstacles with
  | [] -> nan
  | obs :: rest ->
    List.fold_left
      (fun acc obstacle -> min acc (clearance_to_obstacle pos radius obstacle))
      (clearance_to_obstacle pos radius obs)
      rest

let advance_dynamic ~dt (obs : runtime_dynamic) =
  if obs.reached || obs.speed <= 0.0 then
    {
      obs with
      state =
        {
          pos = (if obs.reached then obs.goal else obs.state.pos);
          vel = V3.zero;
        };
      reached = obs.reached;
    }
  else
    let delta = V3.(obs.goal - obs.state.pos) in
    let dist = V3.norm delta in
    if dist <= stop_distance obs.radius then
      { obs with state = { pos = obs.goal; vel = V3.zero }; reached = true }
    else
      let step_len = obs.speed *. dt in
      if step_len +. eps >= dist then
        { obs with state = { pos = obs.goal; vel = V3.zero }; reached = true }
      else
        let vel = toward ~speed:obs.speed obs.state.pos obs.goal in
        {
          obs with
          state = { pos = V3.(obs.state.pos + (dt * vel)); vel };
          reached = false;
        }

let update_agent ~dt params
  (static_obstacles : Scene_config.static_obstacle list)
  (dynamic_obstacles : runtime_dynamic list) (agents : runtime_agent list)
  (agent : runtime_agent) =
  if agent.reached || agent.stalled then
    {
      agent with
      state =
        {
          pos = (if agent.reached then agent.goal else agent.state.pos);
          vel = V3.zero;
        };
      reached = agent.reached;
      stalled = agent.stalled;
    }
  else
    let obstacles =
      List.map obstacle_of_static static_obstacles
      @ List.map obstacle_of_dynamic dynamic_obstacles
      @ List.fold_left
          (fun acc (other : runtime_agent) ->
            if other.id = agent.id then acc else obstacle_of_agent other :: acc
            )
          [] agents
    in
    let has_risk =
      Avoid.has_collision_risk params ~dt ~self_radius:agent.radius agent.state
        obstacles
    in
    let clearance_now = min_clearance agent.state.pos agent.radius obstacles in
    let d_goal_prev = V3.distance agent.state.pos agent.goal in
    let v_cmd =
      Avoid.desired_velocity params ~dt ~self_radius:agent.radius
        ~target:agent.goal agent.state obstacles
    in
    let state_next = Avoid.step ~dt agent.state v_cmd in
    let clearance_next = min_clearance state_next.pos agent.radius obstacles in
    let d_goal_next = V3.distance state_next.pos agent.goal in
    let reached = d_goal_next <= stop_distance agent.radius in
    if reached then
      {
        agent with
        state = { pos = agent.goal; vel = V3.zero };
        reached = true;
        stalled = false;
        stall_steps = 0;
      }
    else
      let progress = d_goal_prev -. d_goal_next in
      let low_progress =
        progress <= stall_progress_threshold ~dt params agent.radius
      in
      let overlap_not_improving =
        match (classify_float clearance_now, classify_float clearance_next) with
        | FP_nan, _ | _, FP_nan | FP_infinite, _ | _, FP_infinite -> false
        | _ ->
          clearance_now < 0.0
          && clearance_next
             <= clearance_now +. stall_clearance_threshold agent.radius
      in
      let stall_steps =
        if overlap_not_improving || (has_risk && low_progress) then
          agent.stall_steps + 1
        else 0
      in
      if stall_steps >= stall_window_steps dt then
        {
          agent with
          state = { pos = agent.state.pos; vel = V3.zero };
          reached = false;
          stalled = true;
          stall_steps;
        }
      else
        {
          agent with
          state = state_next;
          reached = false;
          stalled = false;
          stall_steps;
        }

let write_trace_header oc =
  output_string oc
    "step,t,kind,id,x,y,z,vx,vy,vz,radius,goal_x,goal_y,goal_z,d_goal,min_clearance,reached,stalled\n"

let write_metrics_header oc =
  output_string oc
    "step,t,reached_agents,stalled_agents,total_agents,min_clearance,max_goal_error\n"

let write_trace_row oc ~step ~t ~kind ~id ~(pos : V3.t) ~(vel : V3.t) ~radius
  ~(goal : V3.t) ~d_goal ~min_clearance ~reached ~stalled =
  Printf.fprintf oc "%d,%.6f,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%d,%d\n"
    step t kind id (csv_float pos.x) (csv_float pos.y) (csv_float pos.z)
    (csv_float vel.x) (csv_float vel.y) (csv_float vel.z) (csv_float radius)
    (csv_float goal.x) (csv_float goal.y) (csv_float goal.z) (csv_float d_goal)
    (csv_float min_clearance) (bool_to_int reached) (bool_to_int stalled)

let write_snapshot trace_oc metrics_oc ~step ~t
  (static_obstacles : Scene_config.static_obstacle list)
  (dynamic_obstacles : runtime_dynamic list) (agents : runtime_agent list) =
  List.iter
    (fun (obs : Scene_config.static_obstacle) ->
      write_trace_row trace_oc ~step ~t ~kind:"static" ~id:obs.id ~pos:obs.pos
        ~vel:V3.zero ~radius:obs.radius ~goal:obs.pos ~d_goal:0.0
        ~min_clearance:nan ~reached:true ~stalled:false )
    static_obstacles;
  List.iter
    (fun (obs : runtime_dynamic) ->
      write_trace_row trace_oc ~step ~t ~kind:"dynamic" ~id:obs.id
        ~pos:obs.state.pos ~vel:obs.state.vel ~radius:obs.radius ~goal:obs.goal
        ~d_goal:(V3.distance obs.state.pos obs.goal)
        ~min_clearance:nan ~reached:obs.reached ~stalled:false )
    dynamic_obstacles;
  let agent_clearances =
    List.map
      (fun (agent : runtime_agent) ->
        let obstacles =
          List.map obstacle_of_static static_obstacles
          @ List.map obstacle_of_dynamic dynamic_obstacles
          @ List.fold_left
              (fun acc (other : runtime_agent) ->
                if other.id = agent.id then acc
                else obstacle_of_agent other :: acc )
              [] agents
        in
        let clearance = min_clearance agent.state.pos agent.radius obstacles in
        let d_goal = V3.distance agent.state.pos agent.goal in
        write_trace_row trace_oc ~step ~t ~kind:"agent" ~id:agent.id
          ~pos:agent.state.pos ~vel:agent.state.vel ~radius:agent.radius
          ~goal:agent.goal ~d_goal ~min_clearance:clearance
          ~reached:agent.reached ~stalled:agent.stalled;
        clearance )
      agents
  in
  let reached_agents =
    List.fold_left
      (fun acc (agent : runtime_agent) -> if agent.reached then acc + 1 else acc)
      0 agents
  in
  let stalled_agents =
    List.fold_left
      (fun acc (agent : runtime_agent) -> if agent.stalled then acc + 1 else acc)
      0 agents
  in
  let max_goal_error =
    List.fold_left
      (fun acc (agent : runtime_agent) ->
        max acc (V3.distance agent.state.pos agent.goal) )
      0.0 agents
  in
  let min_clearance =
    List.fold_left
      (fun acc value ->
        match classify_float value with
        | FP_nan | FP_infinite -> acc
        | _ -> min acc value )
      infinity agent_clearances
  in
  Printf.fprintf metrics_oc "%d,%.6f,%d,%d,%d,%s,%s\n" step t reached_agents
    stalled_agents (List.length agents)
    (csv_float
       ( if classify_float min_clearance = FP_infinite then nan
         else min_clearance ) )
    (csv_float max_goal_error)

let print_progress ~t agents metrics_min_clearance =
  let reached_agents =
    List.fold_left
      (fun acc (agent : runtime_agent) -> if agent.reached then acc + 1 else acc)
      0 agents
  in
  let stalled_agents =
    List.fold_left
      (fun acc (agent : runtime_agent) -> if agent.stalled then acc + 1 else acc)
      0 agents
  in
  let anchor =
    match agents with
    | [] -> "none"
    | agent :: _ ->
      Printf.sprintf "%s pos=%s vel=%s stalled=%d" agent.id
        (pp_v3 agent.state.pos) (pp_v3 agent.state.vel)
        (bool_to_int agent.stalled)
  in
  let clearance_text =
    match classify_float metrics_min_clearance with
    | FP_nan | FP_infinite -> "n/a"
    | _ -> Printf.sprintf "%.3f" metrics_min_clearance
  in
  Printf.printf "t=%5.2fs reached=%d stalled=%d total=%d min_clearance=%s anchor=%s\n%!"
    t reached_agents stalled_agents (List.length agents) clearance_text anchor

let run_demo () =
  let config_path =
    if Array.length Sys.argv > 1 then Sys.argv.(1)
    else Scene_config.default_path
  in
  let config =
    try Scene_config.load config_path
    with Scene_config.Config_error msg ->
      Printf.eprintf "Config error: %s\n%!" msg;
      exit 2
  in
  ensure_dir config.trace_path;
  ensure_dir config.metrics_path;
  let trace_oc = open_out config.trace_path in
  let metrics_oc = open_out config.metrics_path in
  write_trace_header trace_oc;
  write_metrics_header metrics_oc;
  let closed = ref false in
  let close_outputs () =
    if not !closed then (
      close_out trace_oc;
      close_out metrics_oc;
      closed := true )
  in
  let static_obstacles = config.static_obstacles in
  let agents = List.map make_runtime_agent config.agents in
  let dynamic_obstacles =
    List.map make_runtime_dynamic config.dynamic_obstacles
  in
  let rec loop step agents dynamic_obstacles =
    let t = config.dt *. float_of_int step in
    write_snapshot trace_oc metrics_oc ~step ~t static_obstacles
      dynamic_obstacles agents;
    flush trace_oc;
    flush metrics_oc;
    let metrics_min_clearance =
      List.fold_left
        (fun acc (agent : runtime_agent) ->
          let obstacles =
            List.map obstacle_of_static static_obstacles
            @ List.map obstacle_of_dynamic dynamic_obstacles
            @ List.fold_left
                (fun acc (other : runtime_agent) ->
                  if other.id = agent.id then acc
                  else obstacle_of_agent other :: acc )
                [] agents
          in
          let clearance =
            min_clearance agent.state.pos agent.radius obstacles
          in
          match classify_float clearance with
          | FP_nan | FP_infinite -> acc
          | _ -> min acc clearance )
        infinity agents
    in
    if step mod 20 = 0 then
      print_progress ~t agents
        ( if classify_float metrics_min_clearance = FP_infinite then nan
          else metrics_min_clearance );
    let all_reached =
      List.for_all (fun (agent : runtime_agent) -> agent.reached) agents
    in
    let all_inactive =
      List.for_all
        (fun (agent : runtime_agent) -> agent.reached || agent.stalled)
        agents
    in
    if all_reached then (
      close_outputs ();
      Printf.printf "All agents reached their goals at t=%.2fs\n%!" t )
    else if all_inactive then (
      let reached_agents =
        List.fold_left
          (fun acc (agent : runtime_agent) -> if agent.reached then acc + 1 else acc)
          0 agents
      in
      let stalled_agents =
        List.fold_left
          (fun acc (agent : runtime_agent) -> if agent.stalled then acc + 1 else acc)
          0 agents
      in
      close_outputs ();
      Printf.printf
        "Simulation ended with %d reached and %d stalled agents at t=%.2fs\n%!"
        reached_agents stalled_agents t )
    else if step >= config.max_steps then (
      close_outputs ();
      Printf.printf "Simulation ended at step limit (%d)\n%!" config.max_steps )
    else
      let agents_next =
        List.map
          (update_agent ~dt:config.dt config.params static_obstacles
             dynamic_obstacles agents )
          agents
      in
      let dynamic_next =
        List.map (advance_dynamic ~dt:config.dt) dynamic_obstacles
      in
      loop (step + 1) agents_next dynamic_next
  in
  try loop 0 agents dynamic_obstacles
  with e ->
    close_outputs ();
    raise e

let () = run_demo ()
