type t = { x : float; y : float; z : float }

val make : float -> float -> float -> t
val zero : t
val ( + ) : t -> t -> t
val ( - ) : t -> t -> t
val ( * ) : float -> t -> t
val scale : float -> t -> t
val div : t -> float -> t
val dot : t -> t -> float
val cross : t -> t -> t
val norm2 : t -> float
val norm : t -> float
val norm_xy : t -> float
val xy : t -> t
val with_z : t -> float -> t
val normalize : ?eps:float -> t -> t
val clamp_norm : float -> t -> t
val distance : t -> t -> float
val is_finite : t -> bool
