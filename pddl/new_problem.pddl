(define (problem task)
(:domain expass2_domain)
(:objects
    wp1 wp2 wp3 wp4 - waypoint
    wphome - home
)
(:init

    (at_home wphome)


    (not (ready_hp))

    (not (new_hint wp1))
    (not (new_hint wp2))
    (not (new_hint wp3))
    (not (new_hint wp4))

)
(:goal (and
    (check_hp)
))
)
