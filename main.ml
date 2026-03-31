let dt = 0.1
let max_steps = 600
let pp_v3 (v : V3.t) = Printf.sprintf "(%.3f, %.3f, %.3f)" v.x v.y v.z

let min_dist_to_obstacles pos obstacles =
  List.fold_left
    (fun acc (o : Avoid.obstacle) ->
      let d = V3.distance pos o.pos in
      min acc d )
    infinity obstacles

let run_demo () =
  let params =
    {
      Avoid.default_params with
      rpz = 1.0;
      max_speed = 0.25;
      tp_samples = 72;
      phi_steps = 11;
    }
  in
  let target = V3.make 3.0 2.5 3.0 in
  let static_obs : Avoid.obstacle =
    { Avoid.pos = V3.make 3.5 (-1.0) 2.0; vel = V3.zero }
  in
  let csv = open_out "sim_trace.csv" in
  Printf.fprintf csv
    "step,t,uav_x,uav_y,uav_z,uav_vx,uav_vy,uav_vz,dyn_x,dyn_y,dyn_z,stat_x,stat_y,stat_z,d_min,rpz,target_x,target_y,target_z\n";
  let closed = ref false in
  let close_csv () =
    if not !closed then (
      close_out csv;
      closed := true )
  in
  let write_row ~step ~t ~(uav : Avoid.state) ~(dyn : Avoid.obstacle) ~d_min =
    Printf.fprintf csv
      "%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n"
      step t uav.Avoid.pos.x uav.Avoid.pos.y uav.Avoid.pos.z uav.Avoid.vel.x
      uav.Avoid.vel.y uav.Avoid.vel.z dyn.Avoid.pos.x dyn.Avoid.pos.y
      dyn.Avoid.pos.z static_obs.Avoid.pos.x static_obs.Avoid.pos.y
      static_obs.Avoid.pos.z d_min params.Avoid.rpz target.x target.y target.z
  in
  let rec loop k (uav : Avoid.state) (dyn : Avoid.obstacle) =
    if k > max_steps then (
      close_csv ();
      Printf.printf "Simulation ended at step limit (%d)\n%!" max_steps;
      () )
    else
      let obstacles = [ static_obs; dyn ] in
      let v_cmd = Avoid.desired_velocity params ~target uav obstacles in
      let uav_next = Avoid.step ~dt uav v_cmd in
      let dyn_next = { dyn with pos = V3.(dyn.pos + (dt * dyn.vel)) } in
      let t = dt *. float_of_int k in
      let d_min = min_dist_to_obstacles uav_next.pos obstacles in
      write_row ~step:k ~t ~uav:uav_next ~dyn ~d_min;
      if k mod 20 = 0 then
        Printf.printf "t=%5.2fs pos=%s vel=%s d_min=%.3f\n%!" t
          (pp_v3 uav_next.pos) (pp_v3 uav_next.vel) d_min;
      if V3.distance uav_next.pos target < 0.25 then (
        close_csv ();
        Printf.printf "Reached target at t=%.2fs, pos=%s\n%!" t
          (pp_v3 uav_next.pos) )
      else loop (k + 1) uav_next dyn_next
  in
  let uav0 : Avoid.state =
    { Avoid.pos = V3.make (-2.0) (-2.0) 1.0; vel = V3.zero }
  in
  let dyn0 : Avoid.obstacle =
    { Avoid.pos = V3.make (-1.5) 1.5 2.5; vel = V3.make 0.10 (-0.12) 0.0 }
  in
  try loop 0 uav0 dyn0
  with e ->
    close_csv ();
    raise e

let () = run_demo ()
