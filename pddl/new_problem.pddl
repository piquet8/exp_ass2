(define (problem task)
(:domain expass2_domain)
(:objects
    wp1 wp2 wp3 wp4 - waypoint
    wphome - home
)
(:init



    (robot_at_home wphome)

    (new_hint wp2)
    (new_hint wp1)
    (new_hint wp4)

)
(:goal (and
    (check_hp)
))
)
