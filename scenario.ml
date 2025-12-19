let max_attempts = 10_000

(* 6 faces of the cube: x=±half, y=±half, z=±half *)
type face = Xpos | Xneg | Ypos | Yneg | Zpos | Zneg

let all_faces = [| Xpos; Xneg; Ypos; Yneg; Zpos; Zneg |]
let rand_face () = all_faces.(Random.int 6)

(* Uniform point on a given face: pick the two free coordinates uniformly in
   [-half, half], fix the remaining coordinate to ±half. *)
let sample_on_face (half : float) (f : face) : V3.t =
  let u () = Random.float (2. *. half) -. half in
  match f with
  | Xpos -> { x = half; y = u (); z = u () }
  | Xneg -> { x = -.half; y = u (); z = u () }
  | Ypos -> { x = u (); y = half; z = u () }
  | Yneg -> { x = u (); y = -.half; z = u () }
  | Zpos -> { x = u (); y = u (); z = half }
  | Zneg -> { x = u (); y = u (); z = -.half }

exception Good

let set_points_uniform (dim : int) (half : float) (min_dist2 : float) :
  V3.t array =
  let tab = Array.init dim (fun _ -> V3.zero) in
  for i = 0 to dim - 1 do
    let attempt = ref 0 in
    try
      while true do
        incr attempt;
        if !attempt > max_attempts then
          failwith
            (Printf.sprintf
               "set_points_uniform: cannot place point %d (dim=%d, radius=%f, \
                min_dist2=%f) after %d attempts."
               i dim half min_dist2 !attempt );

        let f = rand_face () in
        let p = sample_on_face half f in

        (* check separation from previous points *)
        try
          for j = 0 to i - 1 do
            let d2 = V3.norm2 V3.(p - tab.(j)) in
            if d2 < min_dist2 then raise Exit
          done;
          tab.(i) <- p;
          raise Good
        with Exit -> ()
      done
    with Good -> ()
  done;
  tab

let set_points (dim : int) (half : float) (min_dist2 : float) : V3.t array =
  set_points_uniform dim half min_dist2

let set_dests (dim : int) (half : float) (min_dist2 : float) : V3.t array =
  set_points_uniform dim half min_dist2