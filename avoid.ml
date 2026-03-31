type state = { pos : V3.t; vel : V3.t }
type obstacle = { pos : V3.t; vel : V3.t }

type params = {
  rpz : float;
  tp_samples : int;
  phi_steps : int;
  phi_window : float;
  max_speed : float;
  vo_margin : float;
}

let pi = 4.0 *. atan 1.0
let eps = 1e-9

let default_params =
  {
    rpz = 1.0;
    tp_samples = 64;
    phi_steps = 9;
    phi_window = 30.0 *. pi /. 180.0;
    max_speed = 0.25;
    vo_margin = 0.02;
  }

let clamp lo hi x = max lo (min hi x)

let limit_speed vmax v =
  let n = V3.norm v in
  if n <= vmax || n < eps then v else V3.(vmax /. n * v)

let rotate_x phi (v : V3.t) : V3.t =
  let c = cos phi in
  let s = sin phi in
  { x = v.x; y = (c *. v.y) +. (s *. v.z); z = (-.s *. v.y) +. (c *. v.z) }

let rotate_x_inv phi (v : V3.t) : V3.t =
  let c = cos phi in
  let s = sin phi in
  { x = v.x; y = (c *. v.y) -. (s *. v.z); z = (s *. v.y) +. (c *. v.z) }

(* Eq. (8): rotate vector from {c} to {b}. *)
let rotate_c_to_b theta_az theta_el (v : V3.t) : V3.t =
  let caz = cos theta_az in
  let saz = sin theta_az in
  let cel = cos theta_el in
  let sel = sin theta_el in
  {
    x = (caz *. cel *. v.x) +. (-.saz *. v.y) +. (-.caz *. sel *. v.z);
    y = (-.saz *. cel *. v.x) +. (caz *. v.y) +. (-.saz *. sel *. v.z);
    z = (-.sel *. v.x) +. (cel *. v.z);
  }

(* Eq. (4): rotate vector from {b} to {c}. *)
let rotate_b_to_c theta_az theta_el (v : V3.t) : V3.t =
  let caz = cos theta_az in
  let saz = sin theta_az in
  let cel = cos theta_el in
  let sel = sin theta_el in
  {
    x = (caz *. cel *. v.x) +. (saz *. cel *. v.y) +. (-.sel *. v.z);
    y = (-.saz *. v.x) +. (caz *. v.y);
    z = (caz *. sel *. v.x) +. (-.saz *. sel *. v.y) +. (cel *. v.z);
  }

type cone_geom = {
  rvo : float;
  dvo : float;
  theta_az : float;
  theta_el : float;
}

let cone_geometry rpz (p_b : V3.t) =
  let d = V3.norm p_b in
  if d <= rpz +. eps then None
  else
    let d2 = d *. d in
    let base = max 0.0 (d2 -. (rpz *. rpz)) in
    let rvo = rpz *. sqrt base /. d in
    let dvo = base /. d in
    let theta_az = atan2 p_b.y p_b.x in
    let xy = sqrt ((p_b.x *. p_b.x) +. (p_b.y *. p_b.y)) in
    let theta_el = atan2 p_b.z xy in
    Some { rvo; dvo; theta_az; theta_el }

let in_collision_cone params (uav : state) (obs : obstacle) =
  let p_b = V3.(obs.pos - uav.pos) in
  match cone_geometry params.rpz p_b with
  | None -> true
  | Some g ->
    let v_rel_b = V3.(uav.vel - obs.vel) in
    let v_rel_c = rotate_b_to_c g.theta_az g.theta_el v_rel_b in
    if v_rel_c.x <= eps then false
    else
      let lateral =
        sqrt ((v_rel_c.y *. v_rel_c.y) +. (v_rel_c.z *. v_rel_c.z))
      in
      let lhs = lateral /. v_rel_c.x in
      let rhs = g.rvo /. max eps g.dvo in
      lhs < rhs

let tp_samples n =
  let n = max 8 n in
  Array.init (n + 1) (fun i -> 2.0 *. pi *. float_of_int i /. float_of_int n)

let shoelace_area_xy (pts : V3.t list) =
  match pts with
  | [] | [ _ ] | [ _; _ ] -> infinity
  | _ ->
    let arr = Array.of_list pts in
    let n = Array.length arr in
    let sum = ref 0.0 in
    for i = 0 to n - 1 do
      let j = if i = n - 1 then 0 else i + 1 in
      sum := !sum +. ((arr.(i).x *. arr.(j).y) -. (arr.(j).x *. arr.(i).y))
    done;
    abs_float !sum *. 0.5

let angle_to_target (v : V3.t) (t : V3.t) =
  let nv = V3.norm v in
  let nt = V3.norm t in
  if nv < eps || nt < eps then infinity
  else
    let c = clamp (-1.0) 1.0 (V3.dot v t /. (nv *. nt)) in
    acos c

let select_boundary_velocity params ~(target_b : V3.t) (uav : state)
  (obs : obstacle) =
  let p_b = V3.(obs.pos - uav.pos) in
  match cone_geometry params.rpz p_b with
  | None -> None
  | Some g ->
    let tp_grid = tp_samples params.tp_samples in
    let target_y = target_b.y in
    let target_z = target_b.z in
    let phi_target =
      if abs_float target_y < eps then
        clamp
          ((-0.5 *. pi) +. 1e-6)
          ((0.5 *. pi) -. 1e-6)
          (if target_z >= 0.0 then 0.5 *. pi else -0.5 *. pi)
      else atan (target_z /. target_y)
    in
    let lo =
      clamp
        ((-0.5 *. pi) +. 1e-6)
        ((0.5 *. pi) -. 1e-6)
        (phi_target -. params.phi_window)
    in
    let hi =
      clamp
        ((-0.5 *. pi) +. 1e-6)
        ((0.5 *. pi) -. 1e-6)
        (phi_target +. params.phi_window)
    in
    let steps = max 2 params.phi_steps in
    let pick_for_phi phi =
      let a_phi = rotate_x phi obs.vel in
      let target_phi = rotate_x phi target_b in
      let boundary =
        Array.fold_left
          (fun acc tp ->
            let b_vo_c = V3.make g.dvo (g.rvo *. cos tp) (g.rvo *. sin tp) in
            let b_vo_b =
              V3.(rotate_c_to_b g.theta_az g.theta_el b_vo_c + obs.vel)
            in
            let b_phi = rotate_x phi b_vo_b in
            let den = b_phi.z -. a_phi.z in
            if abs_float den < eps then acc
            else
              let tg = a_phi.z /. (a_phi.z -. b_phi.z) in
              if tg < 0.0 then acc
              else
                let inter =
                  V3.make
                    (((b_phi.x -. a_phi.x) *. tg) +. a_phi.x)
                    (((b_phi.y -. a_phi.y) *. tg) +. a_phi.y)
                    0.0
                in
                inter :: acc )
          [] tp_grid
      in
      let boundary = List.rev boundary in
      let area = shoelace_area_xy boundary in
      if classify_float area = FP_infinite then None
      else
        let target_xy = { target_phi with z = 0.0 } in
        let best =
          List.fold_left
            (fun best p ->
              let score = angle_to_target p target_xy in
              match best with
              | None -> Some (score, p)
              | Some (s, q) -> if score < s then Some (score, p) else Some (s, q)
              )
            None boundary
        in
        match best with
        | None -> None
        | Some (_, chosen) ->
          let v_phi = V3.((1.0 +. params.vo_margin) * chosen) in
          Some (area, rotate_x_inv phi v_phi)
    in
    let best = ref None in
    for i = 0 to steps - 1 do
      let alpha =
        if steps = 1 then 0.5 else float_of_int i /. float_of_int (steps - 1)
      in
      let phi = lo +. ((hi -. lo) *. alpha) in
      match pick_for_phi phi with
      | None -> ()
      | Some (area, v) -> (
        match !best with
        | None -> best := Some (area, v)
        | Some (a_best, _) when area < a_best -> best := Some (area, v)
        | _ -> () )
    done;
    Option.map snd !best

let nearest_obstacle (uav : state) (obstacles : obstacle list) =
  List.fold_left
    (fun acc obs ->
      let d = V3.distance uav.pos obs.pos in
      match acc with
      | None -> Some (obs, d)
      | Some (_, d0) when d < d0 -> Some (obs, d)
      | _ -> acc )
    None obstacles

let desired_velocity params ~target (uav : state) (obstacles : obstacle list) =
  let target_b = V3.(target - uav.pos) in
  let v_pref = limit_speed params.max_speed target_b in
  match nearest_obstacle uav obstacles with
  | None -> v_pref
  | Some (obs_near, d_near) ->
    let need_vo = in_collision_cone params uav obs_near in
    if need_vo then
      match select_boundary_velocity params ~target_b uav obs_near with
      | Some v -> limit_speed params.max_speed v
      | None -> v_pref
    else v_pref

let step ~dt (s : state) (v_cmd : V3.t) : state =
  { pos = V3.(s.pos + (dt * v_cmd)); vel = v_cmd }
