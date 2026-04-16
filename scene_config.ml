type static_obstacle = { id : string; pos : V3.t; radius : float }

type moving_obstacle = {
  id : string;
  start : V3.t;
  goal : V3.t;
  speed : float;
  radius : float;
}

type agent = { id : string; start : V3.t; goal : V3.t; radius : float }

type t = {
  dt : float;
  max_steps : int;
  trace_path : string;
  metrics_path : string;
  params : Avoid.params;
  static_obstacles : static_obstacle list;
  dynamic_obstacles : moving_obstacle list;
  agents : agent list;
}

let default_path = "scene.conf"
let pi = 4.0 *. atan 1.0

exception Config_error of string

type partial = {
  dt : float;
  max_steps : int;
  trace_path : string;
  metrics_path : string;
  tp_samples : int;
  phi_steps : int;
  phi_window : float;
  max_speed : float;
  vo_margin : float;
  static_obstacles : static_obstacle list;
  dynamic_obstacles : moving_obstacle list;
  agents : agent list;
}

let default_partial =
  {
    dt = 0.1;
    max_steps = 600;
    trace_path = "sim_trace.csv";
    metrics_path = "sim_metrics.csv";
    tp_samples = Avoid.default_params.tp_samples;
    phi_steps = Avoid.default_params.phi_steps;
    phi_window = Avoid.default_params.phi_window;
    max_speed = Avoid.default_params.max_speed;
    vo_margin = Avoid.default_params.vo_margin;
    static_obstacles = [];
    dynamic_obstacles = [];
    agents = [];
  }

let fail path line_no msg =
  raise (Config_error (Printf.sprintf "%s:%d: %s" path line_no msg))

let strip_comment line =
  let rec find_hash i =
    if i >= String.length line then None
    else if line.[i] = '#' then Some i
    else find_hash (i + 1)
  in
  match find_hash 0 with None -> line | Some idx -> String.sub line 0 idx

let trim = String.trim

let tokenize line =
  line
  |> String.split_on_char ' '
  |> List.map (fun token -> String.split_on_char '\t' token)
  |> List.flatten
  |> List.map trim
  |> List.filter (fun token -> token <> "")

let parse_kv path line_no token =
  match String.split_on_char '=' token with
  | [ key; value ] when key <> "" && value <> "" -> (key, value)
  | _ ->
    fail path line_no
      (Printf.sprintf "invalid token `%s`, expected key=value" token)

let find_required path line_no kvs key =
  match List.assoc_opt key kvs with
  | Some value -> value
  | None -> fail path line_no (Printf.sprintf "missing `%s`" key)

let find_optional kvs key = List.assoc_opt key kvs

let ensure_only_keys path line_no kind kvs allowed =
  List.iter
    (fun (key, _) ->
      if not (List.mem key allowed) then
        fail path line_no
          (Printf.sprintf "unknown key `%s` in `%s` entry" key kind) )
    kvs

let parse_float path line_no key value =
  try float_of_string value
  with Failure _ ->
    fail path line_no
      (Printf.sprintf "`%s` expects a float, got `%s`" key value)

let parse_int path line_no key value =
  try int_of_string value
  with Failure _ ->
    fail path line_no (Printf.sprintf "`%s` expects an int, got `%s`" key value)

let parse_vec3 path line_no key value =
  match String.split_on_char ',' value |> List.map trim with
  | [ x; y; z ] ->
    V3.make
      (parse_float path line_no key x)
      (parse_float path line_no key y)
      (parse_float path line_no key z)
  | _ ->
    fail path line_no (Printf.sprintf "`%s` expects x,y,z, got `%s`" key value)

let validate_unique_ids path kind items =
  let seen = Hashtbl.create 16 in
  List.iter
    (fun id ->
      if Hashtbl.mem seen id then
        raise
          (Config_error
             (Printf.sprintf "%s: duplicate `%s` id `%s`" path kind id) )
      else Hashtbl.add seen id true )
    items

let parse_sim_entry path line_no kvs (cfg : partial) =
  ensure_only_keys path line_no "sim" kvs
    [ "dt"; "max_steps"; "trace"; "metrics" ];
  let dt =
    match find_optional kvs "dt" with
    | Some value -> parse_float path line_no "dt" value
    | None -> cfg.dt
  in
  let max_steps =
    match find_optional kvs "max_steps" with
    | Some value -> parse_int path line_no "max_steps" value
    | None -> cfg.max_steps
  in
  let trace_path =
    match find_optional kvs "trace" with
    | Some value -> value
    | None -> cfg.trace_path
  in
  let metrics_path =
    match find_optional kvs "metrics" with
    | Some value -> value
    | None -> cfg.metrics_path
  in
  { cfg with dt; max_steps; trace_path; metrics_path }

let parse_params_entry path line_no kvs (cfg : partial) =
  ensure_only_keys path line_no "params" kvs
    [ "tp_samples"; "phi_steps"; "phi_window_deg"; "max_speed"; "vo_margin" ];
  let tp_samples =
    match find_optional kvs "tp_samples" with
    | Some value -> parse_int path line_no "tp_samples" value
    | None -> cfg.tp_samples
  in
  let phi_steps =
    match find_optional kvs "phi_steps" with
    | Some value -> parse_int path line_no "phi_steps" value
    | None -> cfg.phi_steps
  in
  let phi_window =
    match find_optional kvs "phi_window_deg" with
    | Some value ->
      parse_float path line_no "phi_window_deg" value *. pi /. 180.0
    | None -> cfg.phi_window
  in
  let max_speed =
    match find_optional kvs "max_speed" with
    | Some value -> parse_float path line_no "max_speed" value
    | None -> cfg.max_speed
  in
  let vo_margin =
    match find_optional kvs "vo_margin" with
    | Some value -> parse_float path line_no "vo_margin" value
    | None -> cfg.vo_margin
  in
  { cfg with tp_samples; phi_steps; phi_window; max_speed; vo_margin }

let parse_static_entry path line_no kvs (cfg : partial) =
  ensure_only_keys path line_no "static" kvs [ "id"; "pos"; "radius" ];
  let obs =
    {
      id = find_required path line_no kvs "id";
      pos = parse_vec3 path line_no "pos" (find_required path line_no kvs "pos");
      radius =
        parse_float path line_no "radius"
          (find_required path line_no kvs "radius");
    }
  in
  { cfg with static_obstacles = obs :: cfg.static_obstacles }

let parse_dynamic_entry path line_no kvs (cfg : partial) =
  ensure_only_keys path line_no "dynamic" kvs
    [ "id"; "start"; "goal"; "speed"; "radius" ];
  let obs =
    {
      id = find_required path line_no kvs "id";
      start =
        parse_vec3 path line_no "start" (find_required path line_no kvs "start");
      goal =
        parse_vec3 path line_no "goal" (find_required path line_no kvs "goal");
      speed =
        parse_float path line_no "speed"
          (find_required path line_no kvs "speed");
      radius =
        parse_float path line_no "radius"
          (find_required path line_no kvs "radius");
    }
  in
  { cfg with dynamic_obstacles = obs :: cfg.dynamic_obstacles }

let parse_agent_entry path line_no kvs (cfg : partial) =
  ensure_only_keys path line_no "agent" kvs [ "id"; "start"; "goal"; "radius" ];
  let agent =
    {
      id = find_required path line_no kvs "id";
      start =
        parse_vec3 path line_no "start" (find_required path line_no kvs "start");
      goal =
        parse_vec3 path line_no "goal" (find_required path line_no kvs "goal");
      radius =
        parse_float path line_no "radius"
          (find_required path line_no kvs "radius");
    }
  in
  { cfg with agents = agent :: cfg.agents }

let finalize path (cfg : partial) =
  if cfg.dt <= 0.0 then
    raise (Config_error (Printf.sprintf "%s: dt must be > 0" path));
  if cfg.max_steps < 0 then
    raise (Config_error (Printf.sprintf "%s: max_steps must be >= 0" path));
  if cfg.max_speed <= 0.0 then
    raise (Config_error (Printf.sprintf "%s: max_speed must be > 0" path));
  if cfg.tp_samples < 8 then
    raise (Config_error (Printf.sprintf "%s: tp_samples must be >= 8" path));
  if cfg.phi_steps < 2 then
    raise (Config_error (Printf.sprintf "%s: phi_steps must be >= 2" path));
  if cfg.agents = [] then
    raise
      (Config_error
         (Printf.sprintf "%s: at least one `agent` entry is required" path) );
  List.iter
    (fun (obs : static_obstacle) ->
      if obs.radius < 0.0 then
        raise
          (Config_error
             (Printf.sprintf "%s: static obstacle `%s` has negative radius" path
                obs.id ) ) )
    cfg.static_obstacles;
  List.iter
    (fun (obs : moving_obstacle) ->
      if obs.radius < 0.0 then
        raise
          (Config_error
             (Printf.sprintf "%s: dynamic obstacle `%s` has negative radius"
                path obs.id ) );
      if obs.speed < 0.0 then
        raise
          (Config_error
             (Printf.sprintf "%s: dynamic obstacle `%s` has negative speed" path
                obs.id ) ) )
    cfg.dynamic_obstacles;
  List.iter
    (fun (agent : agent) ->
      if agent.radius < 0.0 then
        raise
          (Config_error
             (Printf.sprintf "%s: agent `%s` has negative radius" path agent.id)
          ) )
    cfg.agents;
  validate_unique_ids path "static"
    (List.map (fun (obs : static_obstacle) -> obs.id) cfg.static_obstacles);
  validate_unique_ids path "dynamic"
    (List.map (fun (obs : moving_obstacle) -> obs.id) cfg.dynamic_obstacles);
  validate_unique_ids path "agent"
    (List.map (fun (agent : agent) -> agent.id) cfg.agents);
  {
    dt = cfg.dt;
    max_steps = cfg.max_steps;
    trace_path = cfg.trace_path;
    metrics_path = cfg.metrics_path;
    params =
      {
        tp_samples = cfg.tp_samples;
        phi_steps = cfg.phi_steps;
        phi_window = cfg.phi_window;
        max_speed = cfg.max_speed;
        vo_margin = cfg.vo_margin;
      };
    static_obstacles = List.rev cfg.static_obstacles;
    dynamic_obstacles = List.rev cfg.dynamic_obstacles;
    agents = List.rev cfg.agents;
  }

let load path =
  let chan =
    try open_in path with Sys_error msg -> raise (Config_error msg)
  in
  let rec loop line_no cfg =
    match input_line chan with
    | line ->
      let line = strip_comment line |> trim in
      if line = "" then loop (line_no + 1) cfg
      else
        let tokens = tokenize line in
        let kind =
          match tokens with
          | [] -> fail path line_no "empty config line"
          | head :: _ -> head
        in
        let kvs =
          match tokens with
          | [] -> []
          | _ :: rest -> List.map (parse_kv path line_no) rest
        in
        let cfg =
          match kind with
          | "sim" -> parse_sim_entry path line_no kvs cfg
          | "params" -> parse_params_entry path line_no kvs cfg
          | "static" -> parse_static_entry path line_no kvs cfg
          | "dynamic" -> parse_dynamic_entry path line_no kvs cfg
          | "agent" -> parse_agent_entry path line_no kvs cfg
          | _ ->
            fail path line_no
              (Printf.sprintf
                 "unknown entry `%s`, expected sim|params|static|dynamic|agent"
                 kind )
        in
        loop (line_no + 1) cfg
    | exception End_of_file ->
      close_in chan;
      finalize path cfg
  in
  loop 1 default_partial
