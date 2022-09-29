(define (domain expass2_domain)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions :adl)

(:types
	waypoint 
	home
)

(:predicates
	; indicate the actual position of the robot
	(robot_at ?wp - waypoint)
	
	;indicate if the robot is in the home position
    (at_home ?h - home)
    
    ;indicate if a hypothesis is checked
	(check_hp)
	
	;indicate if a hypothesis is ready to be checked
	(ready_hp)
	
	;indicate if a wapoint is visited and a hint is taken
	(new_hint ?wp - waypoint)
)

; Move the robot between the waypoints

(:durative-action goto_waypoint
	:parameters (?from ?to - waypoint)
	:duration ( = ?duration 60)
	:condition (and
		(at start (robot_at ?from)))
	:effect (and
		(at end (robot_at ?to))
		(at start (not (robot_at ?from))))
)

; Move the arm's robot in order to take a hint and marked this waypoint as visited
 
(:durative-action take_hint
	:parameters (?wp - waypoint)
	:duration ( = ?duration 60)
	:condition (and (at start(robot_at ?wp)))
	:effect (and
		(at end (new_hint ?wp))
		)
)

; Move the robot from a waypoint to the home position

(:durative-action moveto_oracle
	:parameters (?from - waypoint ?to - home)
	:duration ( = ?duration 60)
	:condition (at start (robot_at ?from))
	:effect (and
		(at start(not (robot_at ?from)))
		(at end (at_home ?to))
		)
)

; Move the robot from the home to a waypoint

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

; Reports if a hypothesis is ready to be checked

(:durative-action hyp_ready
	:parameters (?h - home)
	:duration ( = ?duration 60)
	:condition (and
		(at start (forall (?wp - waypoint)(new_hint ?wp))))
	:effect (and
		(at end (ready_hp))
		)
)

; Reports if a hypothesis has been checkd and if is the winning one

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

