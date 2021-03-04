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
(battery_enough ?r - robot)
(battery_low ?r - robot)
(normal_mode ?m - mode)
(current_system_mode ?m - mode)
(charging_point_at ?wp - waypoint)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?wp1 ?wp2 - waypoint ?m - mode)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?wp1))
        (over all(battery_enough ?r))
        (over all(normal_mode ?m))
        (over all(current_system_mode ?m))
    )
    :effect (and
        (at start(not(robot_at ?r ?wp1)))
        (at end(robot_at ?r ?wp2))
    )
)

(:durative-action askcharge
    :parameters (?r - robot ?wp1 ?wp2 - waypoint)
    :duration ( = ?duration 5)
    :condition (and
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
