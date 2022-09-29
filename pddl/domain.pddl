(define (domain expass2_domain)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions :adl)

(:types
	waypoint 
	home
)

(:predicates
	(robot_at ?wp - waypoint)
	(at_home ?h - home)
	(check_hp)
	(ready_hp)
	(new_hint ?wp - waypoint)
)

; Move to 
(:durative-action goto_waypoint
	:parameters (?from ?to - waypoint)
	:duration ( = ?duration 60)
	:condition (and
		(at start (robot_at ?from)))
	:effect (and
		(at end (robot_at ?to))
		(at start (not (robot_at ?from))))
)

; Take hint 
(:durative-action take_hint
	:parameters (?wp - waypoint)
	:duration ( = ?duration 60)
	:condition (and (at start(robot_at ?wp)))
	:effect (and
		(at end (new_hint ?wp))
		)
)

; Go to home 
(:durative-action moveto_oracle
	:parameters (?from - waypoint ?to - home)
	:duration ( = ?duration 60)
	:condition (at start (robot_at ?from))
	:effect (and
		(at start(not (robot_at ?from)))
		(at end (at_home ?to))
		)
)

; Move from home 
(:durative-action moveto_hint 
	:parameters (?from - home ?to - waypoint)
	:duration ( = ?duration 60)
	:condition (and
		(at start (at_home ?from)))
	:effect (and
		(at start (not(at_home ?from)))
		(at end (robot_at ?to))
		)
)

; Hyp ready
(:durative-action hyp_ready
	:parameters (?h - home)
	:duration ( = ?duration 60)
	:condition (and
		(at start (forall (?wp - waypoint)(new_hint ?wp))))
	:effect (and
		(at end (ready_hp))
		)
)

; Check hyp
(:durative-action check_hyp 
	:parameters (?h - home)
	:duration ( = ?duration 60)
	:condition (and
		(at start (at_home ?h))
		(at start (ready_hp))
		)
	:effect (and
		(at end (check_hp))
		)
)

)

