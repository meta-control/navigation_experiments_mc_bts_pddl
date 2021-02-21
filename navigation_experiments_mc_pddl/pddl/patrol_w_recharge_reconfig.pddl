(define (domain patrol)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
waypoint
mode
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_at ?r - robot ?wp - waypoint)
(patrolled ?wp - waypoint)
(battery_enough ?r - robot)
(battery_low ?r - robot)
(charging_point_at ?wp - waypoint)
(current_system_mode ?m - mode)
(battery_low_mode ?m - mode)
(normal_mode ?m - mode)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?wp1 ?wp2 - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?wp1))
        (over all(battery_enough ?r))
        )
    :effect (and
        (at start(not(robot_at ?r ?wp1)))
        (at end(robot_at ?r ?wp2))
    )
)

(:durative-action reconfig_system
    :parameters (?r - robot ?m1 ?m2 - mode)
    :duration ( = ?duration 1)
    :condition (and
    )
    :effect (and
        (at start(not(current_system_mode ?m1)))
        (at end(current_system_mode ?m2))
    )
)

(:durative-action patrol
    :parameters (?r - robot ?wp - waypoint ?m - mode)
    :duration ( = ?duration 5)
    :condition (and
        (at start(normal_mode ?m))
        (at start(current_system_mode ?m))
        (at start(robot_at ?r ?wp))
       )
    :effect (and
        (at end(patrolled ?wp))
    )
)

(:durative-action askcharge
    :parameters (?r - robot ?wp1 ?wp2 - waypoint ?m - mode)
    :duration ( = ?duration 5)
    :condition (and
        (at start(battery_low_mode ?m))
        (at start(current_system_mode ?m))
        (at start(robot_at ?r ?wp1))
        (at start(charging_point_at ?wp2))
       )
    :effect (and
        (at start(not(robot_at ?r ?wp1)))
        (at start(robot_at ?r ?wp2))
    )
)

(:durative-action charge
    :parameters (?r - robot ?wp - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?wp))
        (at start(battery_low ?r))
        (at start(charging_point_at ?wp))
    )
    :effect (and
         (at end(not(battery_low ?r)))
         (at end(battery_enough ?r))
    )
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;