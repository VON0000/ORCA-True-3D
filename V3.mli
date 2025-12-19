type t = { x : float; y : float; z : float }

val make : float -> float -> float -> t
val zero : t
val ( + ) : t -> t -> t
val ( - ) : t -> t -> t
val ( * ) : float -> t -> t
val dot : t -> t -> float
val cross : t -> t -> t
val norm2 : t -> float
val norm : t -> float
val normalize : ?eps:float -> t -> t
