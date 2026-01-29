let dim = 30
let radius = 10.0
let half = radius /. 2.
let const_speed = 0.1
let pas = 1.
let safe_range = 2. *. const_speed *. pas
let points = Scenario.set_points dim half safe_range
let dests = Scenario.set_dests dim half safe_range

let () =
  ()
