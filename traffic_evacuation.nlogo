;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Traffic Light Timing for the Evacuation of an Urban Area (TLTEUA)
;; For use with a Genetic Algorithm
;;
;; @author Matt Durak
;; @copyright see bottom
;; @version 5.0.0
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; Measurements:
;; 25 to 35 patches between blocks, average 30 patches
;; Assume 1 tick is 0.5 seconds
;; and a speed of 1 patch per tick is 30 mph
;; This means that 240 patches are 1 mile
;; and each patch is 22 feet
;; Cars are around 12 to 18 feet in length
;; Intersections can be anywhere from 30 seconds to 2 minutes of green

;; PLUME
;; Data from threat at a point from ALOHA
;; measured approximately every block
;; wind from south, source located about 2 blocks south of visible region
;; AEGL data derived

;; ACCELERATION
;; Assuming -23 ft/s/s is a reasonable max brake acceleration
;; @see http://www.batesville.k12.in.us/physics/PhyNet/Mechanics/Kinematics/BrakingDistData.html
;; then -0.2614 per tick should be good
;; @see http://www.jmu.edu/safetyplan/vehicle/generaldriver/stoppingdistance.shtml

;; @todo LIST
;; @version 4.5 Psychology
;;   This version will add driver psychology, including impatience and law breaking
;;   Adding Stress and other states along with methods of emergency information sending (signs/radio etc.)
;;   Remaining tasks:
;;   **car follow: fix model to have reaction time and desired headway
;;   ****aggression: increase reaction time and decrease headway
;;
;;   *difficult tasks that I may skip
;; @version 5.x (supporting NetLogo 5.x)
;;    Use "error" primitive instead of my own EXCEPTION mechanism, this may be cleaner, but it would need to be tested
;;    See if "tasks" (lambdas) could be of any use
;;      NOTE: exporting a world that includes tasks now warns you they won't be re-importable... hmmm
;;    "sort-on" primitive could be useful
;;    Profile everything again, who knows what has changed, ideally you'd want to profile and compare to 4.x
;;      import-world has been improved?


;; Python Preprocessor can strip GUI elements and debug output
;; this block sets up preprocessor variables
;; all elements must start with ";; $@"
;; elements:
;;   -CONFIG: tells preprocessor to read some data for future processing
;;   -END: ends a block until next token
;;   -"NAME" (in config block): defines a new preprocessor name
;;      -TRUE: show (uncomment) all "NAME" blocks
;;      -FALSE: hide (comment) all "NAME" blocks
;;   -"NAME": defines a block that should be commented/uncommented depending on config

;; these settings are for the cluster runs, headless

;;;;;;;;;;;;;;;;
;; $@CONFIG
;; $@VERSION 1
;; $@DEBUG FALSE
;; $@LOGVERBOSE FALSE
;; $@PROFILE FALSE
;; $@GUI FALSE
;; $@PLOTTING FALSE
;; $@END
;;;;;;;;;;;;;;;;

;; Profiling for debugging
;; $@PROFILE
extensions [profiler]
;; $@END

;################################################################################
;; Breeds
;################################################################################

;; keep cars off map as persistant objects
breed [off_map_cars off_map_car]

;; cars and cars turning left are different "breeds"
breed [cars car]
breed [left_turners left_turner]
;; cars that have been created at a garage are a special case
breed [garage_cars garage_car]

;; reuse old cars for memory management
breed [objects object]

;; parking garages with cars to create
breed [garages garage]

;; emergency signs that instruct cars to turn on radio
breed [signs sign]

;################################################################################
;; Variables
;################################################################################

globals
[
  NUM_ROADS_X              ;; number of roads in x direction
  NUM_ROADS_Y              ;; number of roads in y direction
  GRID_X_SIZE              ;; the amount of patches in between two roads in the x direction
  GRID_Y_SIZE              ;; the amount of patches in between two roads in the y direction
  
  num_cars_stopped         ;; the number of cars that are stopped during a single pass thru the go procedure
  cars_off_map             ;; the number of cars not on the visible map
  cars_safe                ;; the number of cars that evacuated
  CAR_TOTAL                ;; the total number of cars
  
  ;; direction constants
  DIR_NORTH
  DIR_EAST
  DIR_SOUTH
  DIR_WEST
  
  ;; light state constants enumeration
  LIGHT_GR
  LIGHT_YR
  LIGHT_RR1
  LIGHT_RG
  LIGHT_RY
  LIGHT_RR2
  
  ;; change_state constants to signify stage in light transition
  STATE_STALE                ;; no change in light timings, business as usual
  STATE_SYNCING              ;; lights have changed but we are waiting for others to change so we are in synchronization mode
  STATE_MID                  ;; lights have made initial swith to red or yellow
  STATE_FRESH                ;; lights have received new timings via file but have not changed
  
  ;; Light timing global constants
  TICKS_ALL_RED              ;; how long the red-red cycle lasts
  
  ;; color constants
  COLOR_NOT_ROAD             ;; color of non-road patches for fast evaluation of road vs. non-road
  COLOR_ROAD                 ;; color of road patches
  COLOR_INTERSECTION         ;; color of intersection patches
  
  ;; MISC Constants
  ;NUM_CARS                    ;; number of cars
  
  ;; Speed control
  SPEED_LIMIT                 ;; standard speed limit (patches/tick)
  SPEED_MEAN                  ;; mean speed
  SPEED_SD                    ;; standard deviation of speed
  SPEED_FLUX                  ;; the maximum amount that the speed limit can change by in range [-SPEED_FLUX/2, SPEED_FLUX/2]
  TURN_SPEED                  ;; speed right turners try to drive to safely make turn
  
  AVG_ACCELERATION            ;; the constant that controls how much a car speeds up or slows down by to accelerate or decelerate
  MAX_ACCELERATION            ;; the acceleration for emergency braking
  
  ;; Alarm ticks/limits
  ;TICKS_TO_ALARM              ;; how long until alarm sounds
  PERCENT_REQUIRED_TO_EVAC    ;; percentage of cars that must evacuate to count the run as "finished"
  TICK_LIMIT                  ;; number of ticks until model should stop
  
  ;; Goal distributions
  PERCENT_LEAVING             ;; percent of cars that have leaving goals
  PERCENT_HEAR_ALARM          ;; percent of cars that hear initial alarm
  PERCENT_HEAR_AFTER          ;; percent of cars that hear the alarm after respawning or meeting a goal
  
  PERCENT_EVAC_TIME_DIV       ;; number to divide evacuation percentages by if they are performed every time step
                              ;; prevents a relatively high probability from becoming more probable over time
  
  ;; Respawning
  TIME_TO_RESPAWN             ;; the time it takes for a car to respawn from the danger zone
  TIME_TO_RESPAWN_MAX         ;; random variance added to respawn time
  
  ;; Decision Percentages
  PERCENT_LANE_CHANGE         ;; percent chance that a car will follow lane change behavior of fast/slow traffic staying left/right
  PERCENT_EVAC_RIGHT          ;; percentage of evacuating cars heading west away from goal that turn right vs. left
  PERCENT_EVAC_TURN           ;; percentage of evacuating cars that will turn when heading up or down before reaching the evac roads
  PERCENT_EVAC_TRY_ROUTE      ;; percentage of evacuating cars heading east that will try to find the evacuation route
  EVAC_ROAD_DIST_MOD          ;; compare the distance to the evac road with the distance to evacuate times this
  PERCENT_LEAVE_TURN          ;; percentage of leaving cars that will make a turn to head toward their goal
  PERCENT_LEAVE_RIGHT         ;; percentage of leaving cars that will turn right when they can turn left
  
  ;; Max wait times
  HIGH_WAIT_TIME_TOLERANCE    ;; the very upper limit when cars decide that they should turn right because they are waiting for so long
  BLOCK_MAX_WAIT              ;; the time a car evacuating will wait between blocks before changing goal to go to evacuation route
  MAX_WAIT_TIME               ;; the maximum wait time a car can have before it is stuck and we should exit
  U_TURN_WAIT_TIME            ;; the time to wait before making a U-Turn
  
  BOX_BLOCK_QUEUE_LIMIT       ;; the upper limit of the queue of the car ahead for checking if a box is blocked
  LEFT_BOX_BLOCK_BUFFER       ;; extra spaces that left turners look ahead to avoid blocking the box
  
  RIGHT_RED_DIST              ;; the distance to look for turning right on red
  LEFT_CHECK_DIST             ;; the distance left turners need to look to complete a turn safely
  
  PASS_DIST_MAX               ;; the maximum distance a car will pass on the left
  PASS_WAIT_TIME              ;; the time a car will wait before passing on the left

  ;; Plume and exposure
  PLUME_START                 ;; time at which data is available for plume chemical concentrations
  PLUME_UPDATE                ;; how often plume concentration levels update
  AIR_EXCHANGE                ;; the rate at which air exchanges with outside and car
  MEAN_SUSCEPT                ;; average susceptibility, as defined by .5 is "normal" and 1 is very susceptible, etc. 
  SD_SUSCEPT                  ;; standard deviation of the susceptibility  
  
  ;; Plume start variation
  PLUME_VARIATION_X_START     ;; start location of plume
  PLUME_START_VARIATION_SIZE  ;; size of possible start location in the horizontal direction
   
  AEGL_1                      ;; C^1.5 * t = k relationship for 50% of population
  AEGL_2
  AEGL_3
  TIME_SCALE_2                ;; time scaling of concentration exposure for AEGL-2
  TIME_SCALE_3                ;; time scaling of concentration exposure for AEGL-3
  
  plume_center                ;; xcor,ycor of approx plume center
  PLUME_POS_TOLERANCE         ;; value to add/subtract from plume_x because of error in estimating distance (depends on knowledge)
  PLUME_SAFE_DISTANCE         ;; safe distance from plume in which cars may drive N/S toward it
  PERCENT_EVAC_CHANGE         ;; percent chance of changing evacuation direction if it makes sense to do so
  
  ;; Knowledge/Awareness and distribution of information
  PERCENT_RADIO_ON            ;; percent of radios initially on
  PERCENT_RADIO_CHANGE        ;; percent of drivers turning radio on or off at a given time
  AWARENESS_DELTA_AVERAGE     ;; average increase in awareness while listening to radio
  AWARENESS_DELTA_SD          ;; standard deviation ^
  MAX_AWARENESS               ;; limit on awareness of a driver
  AWARENESS_THRESHOLD_HIGH    ;; threshold after which a car must evacuate because they know how urgent it is
  AWARENESS_THRESHOLD_LOW     ;; threshold after which a car is likely to be aware of which side of the plume it is on
  KNOW_DELTA_INTERSECTION     ;; The increase in knowledge by driving to a new intersection
  MAX_KNOW                    ;; limit on knowledge
  SIGN_TIME                   ;; how long it takes to read a sign and turn on the radio
  EVAC_ROUTE_KNOW_MIN         ;; the minimum knowledge required to know to find an evac route
  
  STRESS_FADE                 ;; normal decrease in sense of urgency over time in absense of stressors
  STRESS_AWARENESS_INC        ;; increment of sense of urgency if awareness is above threshold
  STRESS_AEGL_INC             ;; increment of sense of urgency if AEGL-1
  STRESS_PEDESTRIANS_INC      ;; increment of sense of urgency if car can see pedestrians passed out due to AEGL-2
  STRESS_WAIT_INC             ;; increment of sense of urgency if car is waiting longer than limit
  STRESS_WAIT_DEC             ;; decrement of sense of urgency if not waiting
  STRESS_WAIT_THRESHOLD       ;; time car will wait before sense of urgency increases
  
  RED_LIGHT_WAIT_MIN          ;; Time a car must wait at a red light before deciding to maybe run it if clear
  MAX_CROSS_CHECK_DIST        ;; maximum distance to look outside intersection
  STRESS_ERROR_RATE           ;; how likely a driver is to make a mistake under stress (given full sense of urgency)
  
  PERCENT_AEGL1_RADIO         ;; percent of cars in AEGL1 that will turn on radios upon smelling chemical (low)
  PERCENT_AEGL2PED_RADIO      ;; percent of cars seeing the effects of AEGL2 on pedestrians that will turn on radio
  PERCENT_AEGL1_AWARE         ;; percent of cars in AEGL1 that become more aware by smell of chemical (with or without radio)
  PERCENT_AEGL2PED_AWARE      ;; percent of cars seeing the effects of AEGL2 on pedestrians that become more aware based on that observation
  
  TIME_LIMIT                  ;; max runtime in seconds
  
  $version$                   ;; XML file version
  VERSION                     ;; stored version to check against XML
  
  ;; patch agentsets
  intersections               ;; agentset containing the patches that CONTROL intersections
  controllers                 ;; agentset of intersections that control the light
  roads                       ;; agentset containing the patches that are roads
  artery_roads                ;; agentset containing the patches that are artery roads
  evac_roads                  ;; agentset containing the patches that are evacuation roads
  
  ;plume_random_offset?        ;; switch to turn on or off moving the plume
  plume_offset                ;; can move the plume by a random amount
  plume_y_offset              ;; can shift the plume further south
  conc_points                 ;; patches that have concentration data
  
  ;; types of roads
  ;; Vertical
  north_list
  south_list
  north_south_list
  artery_list
  ;; Horizontal
  east_list
  west_list
  east_west_list
  evac_list
  ;; intersections
  intersection_matrix     ;; keep track of intersections by coordinates
  
  ;; other road variables
  vertical_scale      ;; list of values for where to place vertical roads
  
  ;; evacuation lights
  changed_lights   ;; keep track of which lights have changed
  change_list      ;; keep track of which lights have changed by row
  
  ;; keep track of accidents
  accident_count   ;; number of accidents which have occurred
  
  ;; Flags
  alarm?           ;; true in an emergency, tells cars to evacuate
  done?            ;; when the model has finished running, used to debug without reporting results multiple times
  evacuated?       ;; set when all cars are either evacuated, disabled, or stuck for > 2min (assuming only 10% still non-disabled)
  EXCEPTION?       ;; allow error conditions to stop the program
  logged?          ;; error was logged
  
  ;; file i/o
  inter_file       ;; file with all intersection data for easy manipulation
  evac_file        ;; file with the timing for the evacuation light timing
  out_file         ;; file to write output to
  garage_file      ;; file describing all of the parking garages
  sign_file        ;; file describing the signs
  conc_file        ;; file describing chemical concentration
  resume_file      ;; file to store state of model and resume
  
  ;; error logging
  log_file         ;; file to log errors to
  run_num          ;; run number for HEEDS runs or just 0
  error_text       ;; stores the error output for printing
  
  ;; visual options to change colors need to be reset
  colors_wrong?    ;; set to true when temporarily changing patch colors, must change back immediately in go
  
  ;seed_decimal     ;; the random seed as a number in the range [-1,1] for HEEDS stochasticity
]

turtles-own
[
  speed          ;; the speed of the turtle
  acceleration   ;; the acceleration
  my_speed_limit ;; the speed the turtle wants to go
  speed_multiplier ;; additional speed when stressed
  wait_time      ;; the amount of time since the last time a turtle has moved
  block_time     ;; the amount of time that the car is waiting between intersections (since it changed its curr_goal)
  passing        ;; flag to help drivers pass in oncoming lane and return
  ahead_passed   ;; flag set when the car ahead has initiated a pass, gives cars "vision".  Set to 30 tick count down
  
  running_red?   ;; flag when a car is running a red light (reset after pcolor!=red)
  
  queue
  
  ;turtle_ahead       ;; the turtle that is ahead of this one and within the specified distance (or nobody)
  ;turtle_ahead_dist  ;; the distance to the turtle ahead (if any)
  
  local_goal     ;; the intersection this turtle wants to get to, can be a list
  curr_goal      ;; the intersection this turtle wants to get to now to work toward goal
  turn_goal      ;; the next turn this turtle wants to take (-90, 0, 90)
  evac?          ;; if the turtle "hears" the alarm and is willing to evacuate
  evac_goal      ;; which direction the turtle is evacuating
  leave?         ;; goal to go off the map.  when it is toroidal, this will be reset possibly after "reaching" goal
  leave_goal     ;; which side of the map to go to
  can_change?    ;; used if the turtle is making a desicion it can change incase of a one way road ahead
  
  sense_of_urgency         ;; how much this car is panicking/stressed [0, 1]
  tunnel_vision_threshold  ;; point at which sense of urgency is high enough to decrease performance in finding good routes
  law_compliance_threshold ;; point at which sense of urgency is high enough to break laws
  speeding_threshold       ;; point at which sense of urgency increases driving speed
  ;panic_threshold          ;; point at which the stress -> panic
  knowledge                ;; how well this car knows his/her way around the city [0, 1]
  awareness                ;; how aware this car is of the emergency {none, something is wrong, need to leave, life is in danger} [0, 1]
  radio?                   ;; is the radio on? (assume it is tuned to emergency broadcast, emergency is broadcast to all frequencies, 
                           ;;  or instructed to set to correct frequency)
  see_sign                 ;; how long this turtle has seen an emergency sign w/info to evacuate
  
  susceptibility    ;; normally distributed susceptibility to gas
  vehicle_conc      ;; internal vehicle concentration
  exposure          ;; exposure to concentration over time
  incapacitated?    ;; is this car stopped due to gas?
  aeglevel          ;; none, AEGL1, AEGL2, or AEGL3
  
  ;; !$@DEBUG
  $trace            ;; look at speed control and why the car is stopping or not
  $goal_change      ;; count how many new goals this car has had
  $intersection_hist;; history of intersections traveled
  ;; !$@END
]

left_turners-own
[
  ready?         ;; when a left turner is in the final phase of the turn and is ready to turn
]

garage_cars-own
[
  
]

off_map_cars-own
[
  time           ;; time it will take for car to virtually "turn around"
]

patches-own
[
  concentration      ;; concentration of chemical in ppm for this patch
  conc_list          ;; list of concentrations over time for key patches
  pedestrian_exposure;; exposure to concentration over time for average pedestrian nearby
  pedestrians_AEGL2? ;; whether the pedestrians in this area would reach AEGL-2 or not
  
  intersection?      ;; true if the patch is at the intersection of two roads
  control?           ;; true if the intersection is the traffic light controller
  controller         ;; the patch that controls the intersection
  light_v            ;; Number representing current color of lights, lights cycle green yellow red
  light_h            ;; vertical and horizontal lights
  hlights            ;; patchset of all horizontal lights
  vlights            ;; patchset of all vertical lights
                     ;; -1 for a non-intersection patches.
  row                ;; the row of the intersection counting from the upper left corner of the
                     ;; world.  -1 for non-intersection patches.
  column             ;; the column of the intersection counting from the upper left corner of the
                     ;; world.  -1 for non-intersection patches.
  phase              ;; the phase for the intersection.  -1 for non-intersection patches.
  state              ;; state of the lights based on phase
  green_ticks_h      ;; h and v are horizontal and vertical
  green_ticks_v      ;; now lights can have their own timings
  yellow_ticks_v     ;; how long the yellow light lasts
  yellow_ticks_h     ;; yellow light in horizontal direction
  new_green_h        ;; The following are for changes in light timings
  new_green_v
  new_yellow_h
  new_yellow_v
  temp_offset         ;; offset for new timing to get lights in phase
  change_state        ;; how far along a light is in changing
  flow_dir            ;; direction of traffic flow (up, right, down, left) a list incase there are multiple directions (intersections)
]

garages-own
[
  capacity            ;; the full "capacity" of cars that it can hold
  car_amount          ;; the actual number of cars in the garage at that point
  driveway            ;; agentset of the patches that are a driveway
  stop_spot           ;; patch that is red signalling stop sign
  turn_dir            ;; the direction new cars must turn onto the road
]

signs-own
[
  vision_width      ;; degree width of cone in which signs can be read
  vision_dist       ;; distance from which a sign can be read
  vision_patches   ;; cached copy of the patches where the sign can be read
]

;################################################################################
;; Setup Procedures
;################################################################################

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Setup
;; @Observer
;;
;; Initial set up
;; sets up the world with all patches and calls reset to set up the turtles
;;
;; @post simulation is ready to run
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to setup
  ;; $@PROFILE
  ;profiler:start
  ;; $@END
  ;; for logging purposes
  print "***BEGIN************************************************************************"
  print date-and-time
  print "********************************************************************************"
  
  set EXCEPTION? false
  set error_text ""
  ;; start timer here
  reset-timer
  ;; only clear special things here that we wouldn't clear in reset
  clear-patches
  clear-turtles
  clear-output
  
  ;; use a random seed based on a decimal number from -1 to 1
  random-seed seed-int
  
  setup-globals ;; gets called twice, what can you do?

  ;; First we ask the patches to draw themselves and set up a few variables
  ;; don't affect the random number generator with this constant setup
  with-local-randomness
  [
    setup-patches
  ]
  
  set-default-shape turtles "car"
  set-default-shape garages "house"
  set-default-shape signs "triangle 2"

  if (NUM_CARS > count roads)
  [
    set error_text (word "***ERROR: There are too many cars for the amount of road.\n"
                       "   The setup has stopped.")
    print error_text
    set EXCEPTION? true
    report-results true
    stop
  ]

  ;; this will set up all of the turtles
  reset
  ;; $@PROFILE
  ;profiler:stop
  ;output-print profiler:report
  ;profiler:reset
  ;; $@END
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Reset
;; @Observer
;;
;; Sets up everything except the world which is pretty static
;; Called initially and on every subsequent run
;;
;; @pre  setup has been run once
;; @post simulation is ready to run
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to reset
  reset-ticks
  ;; fix old garages
  with-local-randomness
  [
    ask garages
    [
      ask driveway
      [
        set pcolor COLOR_NOT_ROAD
      ]
    ]
  ]
  clear-turtles
  clear-all-plots
  clear-drawing
  setup-globals
  
  with-local-randomness
  [
    ;; set up the initial light phases
    setup-controllers
    ;; reset the concentration
    ask patches
    [
      set concentration 0
    ]
    ;; setup the parking garages
    if (garage?)
    [
      setup-garages
    ]
    ;; setup signs
    if (signs?)
    [
      setup-signs
    ]
  ]
  
  ask n-of NUM_CARS roads with [not intersection?]
  [
    sprout-cars 1
    [
      setup-car
      ;; $@GUI
      set-car-color
      ;; $@END
      record-data
    ]
  ]
  
  set CAR_TOTAL (NUM_CARS + (sum [capacity] of garages))
  
  load-state
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Seed Integer
;; @Observer
;;
;; Show the integer random seed used in this run
;;
;; @report the random seed used by this run converted from a decimal number [-2147483648, 2147483647]
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report seed-int
  ;;maximum integer allowed in random-seed so if seed_decimal is [-1,1] it could theoretically pick ANY integer
  report median (list -2147483648 (seed_decimal * 2147483647) 2147483647)
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Setup Globals
;; @Observer
;;
;; Initialize the global variables to appropriate values
;;
;; @post all global contstants are set and global variables are reset
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to setup-globals
  set VERSION "5.0.0"
  
  ;; globals that we allow to change in xml file
  ifelse (#is_gui?)
  [
	set PLUME_VARIATION_X_START 100
	
    ;set NUM_CARS 5000
    set TICK_LIMIT 7200
    ;set TICKS_TO_ALARM 150 ;; car going east to west speed_limit 1.235 covers map in 542 ticks, speed 0.825 is 717 ticks
    
    set PERCENT_LEAVING 0.5
    set PERCENT_HEAR_ALARM 0.5
    set PERCENT_HEAR_AFTER 0.8

    set TIME_LIMIT 0 ;; Overidden in XML, let GUI runs go unlimited
    
    set out_file "output"
    set inter_file "traffic_pre"
    set evac_file "traffic_evac_lights"
    set garage_file "traffic_evac_garages"
    set sign_file "traffic_evac_signs"
    set conc_file "traffic_evac_conc"
    set log_file "error_log"
    set resume_file "traffic_evac_state.csv"
    set run_num 0
  ]
  [
    ;; check xml file since there is a lot it must have up to date
    if ($version$ != VERSION)
    [
      set error_text (word "***ERROR: The XML File is using the incorrect version: " $version$ ", which should be " VERSION)
      print error_text
      set EXCEPTION? true
      report-results true
      stop
    ]
    ;; globals that are in the interface not xml
    set emergency? true
    set garage? true
    set plume? true
    set change_evac_goal_allowed? true
    set plume_random_offset? true
    set knowledge? true
    set radios? true
    set signs? true
    set stress? true
    set accidents? true
    set u_turns? true
  ]
  
  ;; global parameters that should rarely change (so they are removed from xml)
  set SPEED_LIMIT 1
  set SPEED_MEAN 1.1
  set SPEED_SD 0.2
  set SPEED_FLUX 0.1
  set TURN_SPEED 0.8 * SPEED_LIMIT
  ;; don't make ACCELERATION 0.1 since we could get a rounding error and end up on a patch boundary (?)
  set AVG_ACCELERATION 0.099
  set MAX_ACCELERATION 0.27
  
  ;; this is really just how long we wait until we can determine that no more cars are going to move
  set PERCENT_REQUIRED_TO_EVAC 0.9
  
  set TIME_TO_RESPAWN 60
  set TIME_TO_RESPAWN_MAX 120
  set PERCENT_LANE_CHANGE 0.1
  set PERCENT_EVAC_RIGHT 0.75
  set PERCENT_EVAC_TURN 0.8 ;; better to do higher number so cars don't waste time going vertical
  set PERCENT_EVAC_TRY_ROUTE 0.2 ;; lower is better because right now too many cars on evac road
  set EVAC_ROAD_DIST_MOD 0.3 ;; lower is better because cars should not go too far out of their way
  set PERCENT_LEAVE_TURN 0.35
  set PERCENT_LEAVE_RIGHT 0.6
  
  set HIGH_WAIT_TIME_TOLERANCE 240
  set BLOCK_MAX_WAIT 480
  set U_TURN_WAIT_TIME 120
  
  set LEFT_BOX_BLOCK_BUFFER 3
  set BOX_BLOCK_QUEUE_LIMIT 20
  set MAX_WAIT_TIME 480
  set PASS_WAIT_TIME 20
  set RIGHT_RED_DIST 5
  set LEFT_CHECK_DIST 8
  
  ;; the data below is used with susceptibility so it is for 50% * 2
  set AEGL_1 0.5 ;; this is in ppm
  set AEGL_2 182600 * 2
  set AEGL_3 5477700 * 2
  
  set MEAN_SUSCEPT .5
  set SD_SUSCEPT .2
  
  set AIR_EXCHANGE .00097222 ;; 7 exchanges / hour
  set PLUME_START 600 ;; based on file 5 minutes
  set PLUME_UPDATE 60 ;; 30 seconds
  
  set plume_center (list -40 -100) ;; known default but it will be recalculated
  set PLUME_POS_TOLERANCE 40
  set PERCENT_EVAC_CHANGE 0.5
  
  set PERCENT_EVAC_TIME_DIV 10 ;; divide the percent chance because it is evaluated every time step
  
  set PASS_DIST_MAX 3
  
  ;; awareness/knowledge
  set PERCENT_RADIO_ON 0.4
  set PERCENT_RADIO_CHANGE 0.001
  set AWARENESS_DELTA_AVERAGE 0.003
  set AWARENESS_DELTA_SD 0.001
  set MAX_AWARENESS 0.9
  set AWARENESS_THRESHOLD_HIGH 0.75
  set AWARENESS_THRESHOLD_LOW 0.5
  set KNOW_DELTA_INTERSECTION 0.01
  set MAX_KNOW 1.0
  set SIGN_TIME 6
  set EVAC_ROUTE_KNOW_MIN 0.25
  
  set STRESS_FADE 0.0001
  set STRESS_AWARENESS_INC 0.0016
  set STRESS_AEGL_INC 0.0016
  set STRESS_PEDESTRIANS_INC 0.004
  set STRESS_WAIT_INC 0.0004
  set STRESS_WAIT_DEC 0.0002
  set STRESS_WAIT_THRESHOLD 240 ;; 2 minutes
  
  set RED_LIGHT_WAIT_MIN 10 ;; 10 seconds
  set MAX_CROSS_CHECK_DIST 5
  set STRESS_ERROR_RATE 0.3
  
  set PERCENT_AEGL1_RADIO 0.05
  set PERCENT_AEGL2PED_RADIO 0.05
  set PERCENT_AEGL1_AWARE 0.1
  set PERCENT_AEGL2PED_AWARE 0.1
  
  ;; flags set during course of evacuation
  set done? false
  set alarm? false
  set evacuated? false
  set logged? false
  
  ;; just some stuff to display
  set num_cars_stopped 0
  set cars_safe 0
  set cars_off_map 0
  set changed_lights 0
  
  set accident_count 0
  
  set NUM_ROADS_X 8
  set NUM_ROADS_Y 8
  set GRID_X_SIZE world-width / NUM_ROADS_X
  set GRID_Y_SIZE world-height / NUM_ROADS_Y
  
  set PLUME_START_VARIATION_SIZE GRID_X_SIZE * (27 / 40) ;; 27 patches
  
  set PLUME_SAFE_DISTANCE GRID_Y_SIZE * 4 ;; 4 blocks
  
  if-else (plume_random_offset?)
  [
    ;; allows to range 2/3 of a street in from the left or right of the map
    set plume_offset int ( (random PLUME_START_VARIATION_SIZE) + PLUME_VARIATION_X_START )    
    ;; can be shifted up to 1 block further south
    set plume_y_offset int (-(random GRID_Y_SIZE)) - 50
  ]
  [
    set plume_offset 120
    set plume_y_offset -50
  ]
  
  set COLOR_NOT_ROAD 36
  set COLOR_ROAD white
  set COLOR_INTERSECTION 8
  set colors_wrong? false
  
  ;; constants
  set DIR_NORTH 0
  set DIR_EAST 90
  set DIR_SOUTH 180
  set DIR_WEST 270
  
  ;; states
  set LIGHT_GR 0
  set LIGHT_YR 1
  set LIGHT_RR1 2
  set LIGHT_RG 3
  set LIGHT_RY 4
  set LIGHT_RR2 5
  
  set STATE_STALE -1
  set STATE_SYNCING 0
  set STATE_MID 1
  set STATE_FRESH 3
  
  set TICKS_ALL_RED 2
  
  ;; Vertical
  set north_list [2 4]
  set south_list [1 5]
  set north_south_list [6 7]
  set artery_list [0 3]
  ;; Horizontal
  set east_list [3]
  set west_list [1 5]
  set east_west_list [0 4 7]
  set evac_list [2 6]
  
  ;;filenames have been set, check here for accuracy
  let problem ""
  if (not file-exists?  inter_file)
  [
    set problem (word problem " intersection file (" inter_file ")")
  ]
  if (not file-exists?  evac_file)
  [
    set problem (word problem " evacuation file (" evac_file ")")
  ]
  if (not file-exists?  garage_file)
  [
    set problem (word problem " garage file (" garage_file ")")
  ]
  if (not file-exists?  conc_file)
  [
    set problem (word problem " concentration file (" conc_file ")")
  ]
  if (problem != "")
  [
    set error_text (word "****ERROR: Files missing, cannot continue, message was: " problem)
    print error_text
    set EXCEPTION? true
    report-results true
    stop
  ]
end

;################################################################################
;; Road Setup
;################################################################################

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Setup Patches
;; @Observer
;;
;; Set all patch data and create the road structure
;; very slow procedure, only needs to be called initially
;;
;; @pre  setup-globals has been called
;; @post the patches are all set up except for the intersections
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to setup-patches
  
  ;; skeleton "matrix" or nested list of intersection patches
  set intersection_matrix []
  repeat NUM_ROADS_Y
  [
    let temp_list []
    repeat NUM_ROADS_X
    [
      set temp_list lput (no-patches) temp_list
    ]
    set intersection_matrix lput temp_list intersection_matrix
  ]
  
  ;; initialize the patch-owned variables and color the patches to a base-color
  ask patches
  [
    set intersection? false
    set control? false
    set light_v -1
    set light_h -1
    set hlights no-patches
    set vlights no-patches
    set row -1
    set column -1
    set phase -1
    set state -1
    set change_state -1
    set green_ticks_h -1
    set green_ticks_v -1
    set yellow_ticks_h -1
    set yellow_ticks_v -1
    set temp_offset -1
    set pcolor COLOR_NOT_ROAD
    set flow_dir []
    set conc_list []
    set concentration 0
    set pedestrian_exposure 0
    set pedestrians_AEGL2? false
  ]
  
  set controllers no-patches
  
  ;; segregate roads with temporaries
  let north_roads no-patches
  let west_roads no-patches
  let south_roads no-patches
  let east_roads no-patches
  
  ;; skew the skeleton so vertical roads are not placed evenly
  set vertical_scale (list)
  let i 1
  let j 0
  let k 0
  repeat NUM_ROADS_X
  [
    ifelse (k mod 8 = 1)
    [
      set j 0
    ]
    [
      ifelse (k mod 8 = 2)
      [
        set j 0.3
      ]
      [
        ifelse (k mod 8 = 3)
        [
          set j 0.1
        ]
        [
          ifelse (k mod 8 = 4)
          [
            set j 0.2
          ]
          [
            ifelse (k mod 8 = 5)
            [
              set j -0.1
            ]
            [
              ifelse (k mod 8 = 6)
              [
                set j 0.4
              ]
              [
                set j 0
              ]
            ]
          ]
        ]
      ]
    ]
    set vertical_scale lput floor(min-pxcor + GRID_X_SIZE / 2 * (i + j)) vertical_scale
    set i (i + 2)
    set k (k + 1)
  ]
    
  
  ;; skeleton
  let vroads patches with
    [member? pxcor vertical_scale]
  let hroads patches with 
    [(floor((pycor + max-pycor - floor(GRID_Y_SIZE / 2)) mod GRID_Y_SIZE) = 0)]
  
  ;; Artery multiple lane roads
  set artery_roads
  ( patch-set vroads with
    [
      member? floor((pxcor + max-pxcor) / GRID_X_SIZE) artery_list
    ]
  )
  
  ;; evacuation roads
  set evac_roads
  (patch-set hroads with
    [
      member? floor((pycor + max-pycor) / GRID_Y_SIZE) evac_list
    ]
  )
  
  ;; expand to two lanes
  ;; vertical
  ;; one way north
  set north_roads vroads with
    [
      member? floor((pxcor + max-pxcor) / GRID_X_SIZE) north_list
    ]
  ask north_roads
  [
     set north_roads (patch-set north_roads patch-at 1 0)
  ]
  ;; one way south
  set south_roads vroads with
    [
      member? floor((pxcor + max-pxcor) / GRID_X_SIZE) south_list
    ]
  ask south_roads
  [
     set south_roads (patch-set south_roads patch-at 1 0)
  ]
  
  ;; two way
  set south_roads (patch-set south_roads vroads with
    [
      member? floor((pxcor + max-pxcor) / GRID_X_SIZE) north_south_list
    ])
  ask vroads with
    [
      member? floor((pxcor + max-pxcor) / GRID_X_SIZE) north_south_list
    ]
  [
    set north_roads (patch-set north_roads patch-at 1 0)
  ]
  
  set south_roads (patch-set south_roads artery_roads)
  ask artery_roads
  [
    set north_roads (patch-set north_roads patch-at 1 0)
    set artery_roads (patch-set artery_roads patch-at 1 0)
  ]
  ask artery_roads
  [
    if (member? self north_roads)
    [
      set north_roads (patch-set north_roads patch-at 1 0)
      set artery_roads (patch-set artery_roads patch-at 1 0)
    ]
    if (member? self south_roads)
    [
      set south_roads (patch-set south_roads patch-at -1 0)
      set artery_roads (patch-set artery_roads patch-at -1 0)
    ]
  ]
  
  ;; horizontal
  ;; one way east
  set east_roads hroads with
    [
      member? floor((pycor + max-pycor) / GRID_Y_SIZE) east_list
    ]
  ask east_roads
  [
     set east_roads (patch-set east_roads patch-at 0 -1)
  ]
  ;; one way west
  set west_roads hroads with
    [
      member? floor((pycor + max-pycor) / GRID_Y_SIZE) west_list
    ]
  ask west_roads
  [
     set west_roads (patch-set west_roads patch-at 0 -1)
  ]
  
  ;; two way
  set west_roads (patch-set west_roads hroads with
    [
      member? floor((pycor + max-pycor) / GRID_Y_SIZE) east_west_list
    ])
  ask hroads with
    [
      member? floor((pycor + max-pycor) / GRID_Y_SIZE) east_west_list
    ]
  [
    set east_roads (patch-set east_roads patch-at 0 -1)
  ]
  
  set west_roads (patch-set west_roads evac_roads)
  ask evac_roads
  [
    set east_roads (patch-set east_roads patch-at 0 -1)
    set evac_roads (patch-set evac_roads patch-at 0 -1)
  ]
  ask evac_roads
  [
    if (member? self east_roads)
    [
      set east_roads (patch-set east_roads patch-at 0 -1 patch-at 0 -2)
      set evac_roads (patch-set evac_roads patch-at 0 -1 patch-at 0 -2)
    ]
    if (member? self west_roads)
    [
      set west_roads (patch-set west_roads patch-at 0 1 patch-at 0 2)
      set evac_roads (patch-set evac_roads patch-at 0 1 patch-at 0 2)
    ]
  ]
  
  ;; combined agentsets
  set roads (patch-set south_roads north_roads east_roads west_roads)
  set vroads (patch-set south_roads north_roads)
  set hroads (patch-set east_roads west_roads)
  
  set intersections roads with
  [
    member? self hroads and member? self vroads
  ]
      
  ;; set the direction of the roads
  ask south_roads [set flow_dir lput DIR_SOUTH flow_dir] 
  ask north_roads [set flow_dir lput DIR_NORTH flow_dir] 
  ask east_roads [set flow_dir lput DIR_EAST flow_dir] 
  ask west_roads [set flow_dir lput DIR_WEST flow_dir] 
  
  reset-patch-colors
  
  setup-intersections
  
  ;; set control center to top left of intersection
  set controllers no-patches
  let x 0
  while [x < NUM_ROADS_X]
  [
    let y 0
    while [y < NUM_ROADS_Y]
    [
      ;; grab the control center
      let center first (sort intersections with [row = x and column = y])
      ;; add the controller to the agentset
      set controllers (patch-set controllers center)
      ask center
      [
        set control? true
        find-lights
      ]
      ;; make all other intersections have a reference to their control
      ask intersections with [row = x and column = y]
      [
        set controller center
      ]
      set y y + 1
    ]
    set x x + 1
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Setup Plume
;; @Observer
;; @ask : patches
;;
;; sets up chemical plume data from initial input file, intermediate concentrations must be interpreted
;;
;; @pre  roads are setup, ticks has reached PLUME_START
;; @post patches have chemical concentration data vs. time
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to setup-plume
  ;; reset
  ask patches
  [
    set concentration 0
    set conc_list []
    set pedestrian_exposure 0
    set pedestrians_AEGL2? false
  ]
  ;; keep track of key points
  set conc_points no-patches
  file-open conc_file
  
  while [not file-at-end?]
  [
    let y file-read
    let x file-read
    let $ file-read
    let line file-read-line
    ; don't allow toroidal wrap around with y_offset
    if (y + plume_y_offset >= 0)
    [
      ask (patch-set patch (min-pxcor + x + plume_offset) (min-pycor + y + plume_y_offset) patch (min-pxcor - x + plume_offset) (min-pycor + y + plume_y_offset))
      [
        set conc_list read-from-string line
        set conc_points (patch-set conc_points self)
      ]
    ]
  ]
  
  file-close
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Reset Patch Colors
;; @Observer
;;
;; Sets patch colors back to standard, which is necessary when temporarily viewing chemical concentrations
;;
;; @pre  roads are setup
;; @post roads, lights, intersections, and everything else have correct colors
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to reset-patch-colors
  ask patches
  [
    set pcolor COLOR_NOT_ROAD
  ]
  ask roads
  [
    set pcolor COLOR_ROAD
  ]
  ask intersections
  [
    set pcolor COLOR_INTERSECTION
  ]
  ask controllers
  [
    set-signal-colors
  ]
  ask garages
  [
    ask driveway
    [
      set pcolor white
    ]
    ask stop_spot
    [
      set pcolor red
    ]
  ]
  set colors_wrong? false
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Setup Garages
;; @Observer
;;
;; Reads the input file describing where to place garages and their capacity
;; Creates the garages so that they can spawn cars during an emergency
;;
;; @pre  roads are setup
;; @post Garages have been created
;; @throw shows an error if the file could not be read
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to setup-garages
  ;; find the center of every patch and store it in a list
  let block_centers get-block-centers
  let x 0
  let y 0
  
  ifelse (file-exists? garage_file)
  [
    ;; read the file describing garages
    file-open garage_file
    ;; temporary file read variable
    let temp ""
    ;; get to start of description
    while [temp != "START"]
    [
      set temp file-read
    ]
    
    while [temp != "END" and not file-at-end?]
    [
      set temp file-read
      if (temp != "END")
      [
        ;; read this garage entry
        set x file-read
        set y file-read
        ask (item x (item y block_centers))
        [
          sprout-garages 1
          [
            set shape "house"
            set color gray
            set size 2
            set capacity file-read
            set car_amount capacity
            ;; show the car-count on the screen
            set label car_amount
            set label-color black
            ;; rotate to correct direction
            set heading file-read
            ;; turn right for the offset
            right 90
            jump file-read
            left 90
            ;; move to the road
            while [[pcolor] of patch-ahead 1 != white]
            [
              fd 1
            ]
            ;; get the direction cars must turn onto the road
            set turn_dir (subtract-headings ([first flow_dir] of patch-ahead 1) heading)
            let dist_back file-read
            ;; get the "driveway"
            set driveway patch-here
            set stop_spot patch-here
            ;; make the patch near the road a red light like a stop sign
            ask stop_spot
            [
              set pcolor red
            ]
            let i 1
            while [i <= dist_back]
            [
              set driveway (patch-set driveway patch-ahead (- i))
              set i i + 1
            ]
            ask driveway
            [
              if (pcolor != red)
              [
                set pcolor white
              ]
              set flow_dir lput [heading] of myself flow_dir
            ]
            jump (- dist_back)
          ]
        ]
      ]
    ]
    file-close
  ]
  [
    set error_text (word "***ERROR: Could not open file " garage_file)
    print error_text
    set EXCEPTION? true
    report-results true
    stop
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Setup Signs
;; @Observer
;;
;; Reads the input file describing where to place signs and their properties
;; Creates the signs so that they can inform cars during an emergency
;;
;; @pre  roads are setup
;; @post Signs have been created
;; @throw shows an error if the file could not be read
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to setup-signs
  ;; find the center of every patch and store it in a list
  let block_centers get-block-centers
  let x 0
  let y 0
  
  ifelse (file-exists? sign_file)
  [
    ;; read the file describing signs
    file-open sign_file
    ;; temporary file read variable
    let temp ""
    ;; get to start of description
    while [temp != "START"]
    [
      set temp file-read
    ]
    
    while [temp != "END" and not file-at-end?]
    [
      set temp file-read
      if (temp != "END")
      [
        ;; read this sign entry
        set x file-read
        set y file-read
        ask (item x (item y block_centers))
        [
          sprout-signs 1
          [
            set color red
            set size 4
            ;; face correct direction to road in order to move toward it
            set heading file-read
            ;; move to the road
            while [[pcolor] of patch-ahead 1 != white]
            [
              fd 1
            ]
            ;; gather offset
            let parallel_offset file-read
            let road_offset file-read
            back road_offset
            ;; true direction of sign
            set heading file-read
            fd parallel_offset
            
            ;; read other data
            set vision_width file-read
            set vision_dist file-read
            
            set vision_patches patches in-cone vision_dist vision_width
            
          ]
        ]
      ]
    ]
    file-close
    
;    ;; test
;    ask signs
;    [
;      ask patches in-cone vision_dist 60 [set pcolor yellow]
;      ask patches in-cone vision_dist 45 [set pcolor green]
;      ask patches in-cone vision_dist 30 [set pcolor blue]
;    ]
  ]
  [
    set error_text (word "***ERROR: Could not open file " sign_file)
    print error_text
    set EXCEPTION? true
    report-results true
    stop
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Get Block Centers
;; @Observer
;;
;; Make a list of the center of every block for tasks that need it.
;;
;; @pre  roads are setup
;; @report returns a list of all block centers
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report get-block-centers
  ;; find the center of every block and store it in a list
  let block_centers (list)
  let x 0
  let y 0
  let controller_list sort controllers
  ;; build a list of lists
  repeat (NUM_ROADS_Y)
  [
    set x 0
    let this_row (list)
    ;; build this list
    repeat (NUM_ROADS_X)
    [
      let temp_main 0
      let temp_left 0
      let temp_up 0
      set temp_main item (y * NUM_ROADS_Y + x) controller_list
      ifelse (x = 0)
      [
        set temp_left patch min-pxcor ([pycor] of temp_main)
      ]
      [
        set temp_left item (y * NUM_ROADS_Y + x - 1) controller_list
      ]
      ifelse (y = 0 )
      [
        set temp_up patch ([pxcor] of temp_main) max-pycor
      ]
      [
        set temp_up item ((y - 1) * NUM_ROADS_Y + x) controller_list
      ]
      set this_row lput (patch (int (([pxcor] of temp_main + [pxcor] of temp_left) / 2)) (int (([pycor] of temp_main + [pycor] of temp_up) / 2))) this_row
      set x x + 1
    ]

    set block_centers lput this_row block_centers
    set y y + 1
  ]
  
;  ;; test
;  foreach block_centers
;  [
;    foreach ?
;    [
;      ask ?
;      [
;        set pcolor blue
;      ]
;    ]
;  ]
  
  
  report block_centers
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Read Intersection Data
;; @Observer
;;
;; Initialize traffic light data and timings from a file
;;
;; @pre  inter_file is set, setup-patches has been called
;; @post traffic light timings are set
;; @throw shows an error if the file could not be read
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to read-intersection-data
  ifelse (file-exists? inter_file)
  [
    file-open inter_file
    foreach sort controllers
    [
      ask ?
      [
        set phase file-read
        set green_ticks_h file-read
        set yellow_ticks_h file-read
        set green_ticks_v file-read
        set yellow_ticks_v file-read
        ;; reset some values
        set temp_offset 0
        set change_state STATE_STALE
        set new_green_h 0
        set new_green_v 0
        set new_yellow_h 0
        set new_yellow_v 0
      ]
    ]
    file-close
  ]
  [
    set error_text (word "***ERROR: Could not open file " inter_file)
    print error_text
    set EXCEPTION? true
    report-results true
    stop
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Setup Intersections
;; @Observer
;;
;; Initialize all patch data for intersection patches
;;
;; @pre  setup-patches
;; @post all intersections have basic data set, still must call other functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to setup-intersections
  ask intersections
  [
    set intersection? true
    set control? false
    set light_v red
    set light_h green
    set phase 0
    set temp_offset -1
    set row floor((pycor + max-pycor) / GRID_Y_SIZE)
    set column get-column
    
    ;; add me to the matrix
    ;; replace the current row...
    set intersection_matrix replace-item row intersection_matrix
    ;; ... with the same row with the column replaced...
                                         replace-item column (item row intersection_matrix)
    ;; ... by adding self to that agentset
                                                      (patch-set self (item column (item row intersection_matrix)))
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Get Column
;; @Patch : Intersection
;;
;; Find the column number of unevenly spaced roads
;;
;; @pre  setup-patches
;; @report the column number of the road starting at 0
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report get-column
  let num pxcor
  let i 1
  let j 1
  while [true]
  [
    ifelse (member? num vertical_scale)
    [
      report position num vertical_scale
    ]
    [
      set num (num + i * j)
      set i (i + 1)
      set j (- j)
    ]
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Setup Controllers
;; @Observer
;;
;; Read in initial intersection traffic light data
;; and set up the signals
;;
;; @post intersections are initialized and ready to use
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to setup-controllers
  read-intersection-data
  set-signals
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Find Lights
;; @Patch : Controller
;; run by each controller
;;
;; Cache the location of all of the traffic lights a controller must manage
;; This will look at all surrounding patches at the intersection and find the ones that must be lights
;; Much faster than finding lights every time they must be changed
;;
;; @pre  setup-patches has been called
;; @post the controller patches can now control their lights
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to find-lights
  ;; temporary grab this patches variables for color
  let top_left self
  ;; look at all possible lights and the rest of the intersection
  let lights patch-set neighbors with [pcolor = COLOR_ROAD]
  let inters patch-set neighbors with [ intersection? ]
  ;; count the intersection
  let n count inters 
  
  ;; loop through until all POSSIBLE lights are found
  while [ n >= 0 ]
  [
    ask inters
    [
      set lights (patch-set lights neighbors with [pcolor = COLOR_ROAD])
      set inters (patch-set inters neighbors with [ intersection? ])
    ]
    ;; this will let this run an extra time
    ifelse n != 0 and count inters > n
    [
      set n count inters
    ]
    [
      set n -1
    ]
  ]
  
  set hlights patch-set lights with
  [
    (pxcor < [pxcor] of top_left and member? DIR_EAST flow_dir) or
    (pxcor > [pxcor] of top_left and member? DIR_WEST flow_dir)
  ]
  
  set vlights patch-set lights with
  [
    (pycor > [pycor] of top_left and member? DIR_SOUTH flow_dir) or
    (pycor < [pycor] of top_left and member? DIR_NORTH flow_dir)
  ]
end

;################################################################################
;; Car Setup
;################################################################################

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Setup Car
;; @Turtle
;;
;; Initialize the turtle variables to appropriate values
;; Will set the heading of the turtle to be correct for the lane
;; Sets speed limit randomly based on percentage of fast and slow cars
;; Sets up a random goal and makes the first decision for that goal
;;
;; @pre  turtle has been placed on a set up world on an empty road
;; @post turtle is completely set up and ready to be used in simulation
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to setup-car
  set speed 0
  set acceleration AVG_ACCELERATION
  set wait_time 0
  set block_time 0
  set queue 0
  set can_change? false
  set passing false
  set ahead_passed 0
  
  set running_red? false
  
  ;set turtle_ahead nobody
  ;set turtle_ahead_dist 0
  
  ;; speed limit set to normal, some increased or decreased
  set my_speed_limit random-normal SPEED_MEAN SPEED_SD
  ;; limit min to 0.5 and max to 2.0
  set my_speed_limit median (list 0.5 my_speed_limit 2.0)
  
  set heading [first flow_dir] of patch-here
  
  set local_goal []
  set curr_goal []
  set leave? false
  set evac? false
  set evac_goal 0
  set turn_goal 0
  set leave_goal 0
  
  ;; awareness and knowledge
  set sense_of_urgency 0
  set tunnel_vision_threshold median (list 0.2 (random-normal 0.8 0.1) 0.999)
  set law_compliance_threshold median (list 0.1 (random-normal 0.6 0.1) 0.999)
  set speeding_threshold median (list 0.2 (random-normal 0.6 0.1) 1)
  set speed_multiplier median (list 0 (random-normal 0.11 0.05) 0.3)
  set knowledge ifelse-value (knowledge?) [random-float MAX_KNOW] [0.5] ;; if disabled, set to 0.5
  set awareness 0
  set radio? ifelse-value (random-float 1 < PERCENT_RADIO_ON) [true] [false]
  set see_sign 0
  
  ;; chemical exposure levels
  set susceptibility median (list 0.1 (random-normal MEAN_SUSCEPT SD_SUSCEPT) 0.9)
  set vehicle_conc 0
  set exposure 0
  set incapacitated? false
  set aeglevel 0
  
  ;; $@DEBUG
  set $trace 0
  set $goal_change 0
  set $intersection_hist []
  ;; $@END
  
  ;; figure out goals!
  if (breed = cars)
  [
    ;; only true cars do this
    find-new-goal
    find-first-goal
    update-goal true
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Find First Goal
;; @Turtle
;;
;; Based on the location of the turtle, find the next intersection that it will drive to
;; Does not count the current intersection if the turtle is already on one
;;
;; @pre  setup-turtle and all prerequisites to set up the world
;; @post turtle knows which intersection it is heading to (curr_goal)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to find-first-goal
  ;; store the patch in search for next intersection
  let p patch-here
  let dir heading
  if ([intersection?] of p)
  [
    if (breed = left_turners)
    [
      ;; need to turn left
      let i 1
      while [[intersection?] of (patch-ahead i)]
      [
        ask p
        [
          set p patch-at-heading-and-distance dir 1
        ]
        set i (i + 1)
      ]
      ;; after this, p is the last patch in the intersection, now turn left
      set dir (dir - 90) mod 360
    ]
    ;; already on an intersection, just keep going to the next one, 
    ;; then continue normal loop as if turtle is that far past the intersection
    while [[intersection?] of p]
    [
      ask p
      [
        set p patch-at-heading-and-distance dir 1
      ]
    ]
  ]
  while [[not intersection?] of p]
  [
    ;; @todo this is SLOWWWW
    ask p
    [
      set p patch-at-heading-and-distance dir 1
    ]
  ]
  
  set curr_goal list [column] of p [row] of p
end


;################################################################################
;; Runtime Procedures
;################################################################################

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Go
;; @Observer
;;
;; Run one tick of the simulation
;; Used in forever button
;;
;; @pre setup done correctly
;; @post stops forever button when all cars have evacuated or other defined target reached
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to go
  ;; stop the run when a fatal error occurs
  if EXCEPTION?
  [
    print "An exception was thrown, a detailed message should have been printed..."
    log-error
    stop
  ]
  ;if ticks = 1860 [stop]
  if (colors_wrong?)
  [
    reset-patch-colors
    set colors_wrong? false
  ]
  
  set num_cars_stopped 0
  
  ;; alarm sounds after x ticks
  if (ticks = TICKS_TO_ALARM and emergency?)
  [
    sound-alarm
  ]
  
  ;; update the light phase
  next-phase
  ;; have the intersections change their color
  set-signals
  
  if (alarm?)
  [
    if (not evacuated? and evacuation-done? and emergency? and not done? and CAR_TOTAL > 0)
    [
      set evacuated? true
      print (word "Done evacuating at " ticks "ticks and " timer "seconds, just evaluating now")
    ]
    
    ;; move off map and garage cars back
    respawn-cars
    
    ;; update chemical
    if (ticks = PLUME_START and plume?)
    [
      setup-plume
    ]
    if (ticks >= PLUME_START and plume?)
    [
      update-plume
    ]
    
    ;; synchronize lights
    if (changed_lights < NUM_ROADS_X * NUM_ROADS_Y)
    [
      ask controllers
      [
        sync-timing
      ]
    ]
    ;; countdown for off_map cars
    ask off_map_cars
    [
      set time time - 1
    ]
    
    ;; this will mark the cars that can see the signs and should increase in awareness
    if (signs?)
    [
      ask signs
      [
        update-car-sign-vision
      ]
    ]
    
    ;; exposure calculation and driver awareness
    ask (turtle-set cars left_turners garage_cars)
    [
      update-concentration
      check-aegl
      update-awareness-knowledge
      update-sense-of-urgency
    ]
    ask garages
    [
      update-concentration
    ]
    
  ]
  
  ;; stop doing this if cars are all "evacuated" (or stuck or disabled)
  if (not evacuated?)
  [
    foreach sort-by [[queue] of ?1 < [queue] of ?2] cars
    [
      ask ?
      [
        find-queue
      ]
    ]
    
    ;; These procedures are run by the first car "in line" and continue in order
    ;; left_turners go before normal cars
    ;; cars decide what to do to complete their current goal
    ;; (need to change lanes, make a turn, cancel a turn
    ;; then they make a new decision if needed because a temporary turning goal is met
    ;; then they set their speed based on traffic and lights
    ;; next they change lanes for faster travel
    ;; cars on the edges of the map will "disappear" during an evacuation
    ;; they finally move forward, record some data, and set their color to show speed
    foreach sort-by [
      ([breed] of ?1 = left_turners and [breed] of ?2 = cars) or
      ([breed] of ?1 = [breed] of ?2 and
        [queue] of ?1 < [queue] of ?2)]
    (turtle-set cars left_turners)
    [
      ask ?
      [
        if-else (not incapacitated?)
        [
          ifelse (breed = cars)
          [
            make-turn-decision
            update-goal false
            change-lanes
            update-speed
            maybe-make-u-turn
            apply-acceleration
            jump speed
          ]
          [
            ;; breed = left_turners
            update-goal false
            continue-left-turn
          ]
          if (alarm?)
          [
            check-bounds
          ]
          record-data
          ;; $@GUI  
          set-car-color
          ;; $@END
        ]
        [
          if (speed > 0)
          [
            ;; hit the brakes when incapacitated
            brake MAX_ACCELERATION
            apply-acceleration
            jump speed
          ]
        ]
      ]
    ]
    
    ask cars with [not incapacitated?]
    [
      check-for-collision
    ]
    
    foreach sort-by [[queue] of ?1 < [queue] of ?2] garage_cars
    [
      ask ?
      [
        move-garage-car
        record-data
        ;; $@GUI
        set-car-color
        ;; $@END
      ]
    ]
    
    ;; $@DEBUG
    #check-invariants
    ;; $@END

  ]

  ;; update ticks
  tick
  
  ;; when time limit reached (skip for non-emergency runs)
  if (ticks >= TICK_LIMIT and 
    emergency? and 
    not done? and 
    CAR_TOTAL > 0)
  [
    report-results false
    stop
  ]
  
  if (TIME_LIMIT > 0 and timer > TIME_LIMIT)
  [
    ;; model run went over time limit and we must end it
    ;; let's assume it was some kind of error to be investigated
    set error_text (word "*** Time Limit reached at " ticks " ticks***")
    print error_text
    ;report-results true
    set EXCEPTION? true
    
    ;; store current model state
    save-state
    
    stop
  ]
  
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Check Invariants
;; @Observer
;; @DEBUG
;;
;; Checks a few things which would indicate bugs, used only during debugging because of performance issues
;;
;; @post may cause program to stop and print error message
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to #check-invariants
  
  ;; $@DEBUG
;    ;; DEBUG
;    with-local-randomness
;    [
;      let bug? false
;      ask turtles
;      [
;        if (not #in-sync?)
;        [
;          inspect self
;          print who
;          set bug? true
;        ]
;      ]
;      if bug?
;      [
;        set error_text (word "***ERROR: Out of sync bug has returned! ticks:" ticks)
;        print error_text
;        set EXCEPTION? true
;        report-results true
;        stop
;      ]
;    ]
;; $@END
  
  ;; $@DEBUG  
;    ;; check for "blocked box"
;    let exit false
;    with-local-randomness
;    [
;      if (any? (cars-on intersections) with [wait_time > 5])
;      [
;        print (word "    " ticks ": block: " 
;          ([who] of ((cars-on intersections) with [wait_time > 5])) ", max time: "
;          max [wait_time] of ((cars-on intersections) with [wait_time > 5])
;        )
;      ]
;      if (any? ((cars-on intersections) with [wait_time > MAX_WAIT_TIME]) and not done?)
;      [
;        ;; someone is stuck, let's not waste time
;        set exit true
;        set error_text (word "***ERROR @" ticks ": Stuck cars!")
;        print error_text
;        set EXCEPTION? true
;        report-results true
;      ]
;    ]
;    if (exit)
;    [
;      set exit false
;      stop
;    ]
  ;; $@END
    
  ;; $@DEBUG
;  if (any? patches with [count cars-here > 4])
;  [
;    ask patches with [count cars-here > 4]
;    [
;      set pcolor blue
;    ]
;    print "check it out"
;    stop
;  ]
  ;; $@END
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; In Sync
;; @Turtle
;; @DEBUG
;;
;; Runs find-first-goal continuously to find out when the turtle's
;; curr_goal variable gets out of sync
;; Very slow to run every time, DEBUG ONLY
;;
;; @pre  turtles should have run other routines and have a curr_goal
;; @post will reset the turtle's curr_goal but report false if it has changed
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report #in-sync?
  if (not [intersection?] of patch-here)
  [
    let x first curr_goal
    let y item 1 curr_goal
    find-first-goal
    if (x != first curr_goal or y != item 1 curr_goal)
    [
      type ticks type ": " type x type ", " type y type " " show self
      report false
    ]
  ]
  report true ;; intersections should be safe...for now
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Find Queue
;; @Turtle
;;
;; Determine the turtle's position in line with other turtles
;; This "queue" number is set to 0 or 1 more than the turtle ahead
;;
;; @pre  turtles should be sorted by queue number so that the turtle
;; ahead of this one has the correct queue number
;; @post this turtle should have an accurate number of where it is in line (queue)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to find-queue
  let turtle_ahead find-nearest-car safe-distance heading patch-here
  ifelse (turtle_ahead != nobody)
  [
    set queue ([queue] of turtle_ahead) + 1
  ]
  [
    set queue 0
  ]  
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Crash
;; @Cars
;;
;; What happens when a car has crashed with another
;;
;; @pre Conditions occured for an accident
;; @post car becomes incapacitated only if accidents are enabled
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to crash
  if (accidents? and not incapacitated?)
  [
    set incapacitated? true
    set speed 0
    ;; $@DEBUG
    show (word "I crashed at: " ticks)
    print "########################################  CRASHED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    ;; $@END
    set accident_count accident_count + 1
  ]
end

;################################################################################
;; Speed Control
;################################################################################

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Update Speed
;; @Turtle
;;
;; @todo cars move SLOW, they rarely collide, but don't really accelerate when they should
;;
;; The car will look ahead based on its speed to see cars ahead and lights ahead
;; it will slow down or brake (slow down with more acceleration) if need be
;; the car will speed up if at all possible
;;
;; @post car speed will change 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to update-speed
  let turtle_ahead find-nearest-car safe-distance heading patch-here
  ;; store some function results
  let brake_dist brake-distance
  ;; keep track of whether we need to slow down to do it last
  let slow_down? false
  ;; keep track of how much to slow down
  let brake? false
  let coast? false
  
  ;; running red light, reset flag if no longer on red light and can go back to normal rules
  if (pcolor != red)
  [
    set running_red? false
  ]
  
  ;; $@DEBUG
  set $trace 0
  ;; $@END
  
  ;; look at cars ahead and slow down if necessary
  if (turtle_ahead != nobody)
  [
    let data (speed-car-ahead turtle_ahead)
    set coast? first data
    set slow_down? item 1 data
    set brake? last data
  ]
  
  if (not slow_down?)
  [
    ;; slow down to turn right
    if (
      turn_goal = 90 and 
      right-turn-lane? and 
      intersection-ahead? turn-distance and
      speed > TURN_SPEED
      )
    [
      ;; $@DEBUG
      set $trace $trace + 128
      ;; $@END
      set slow_down? true
    ]
  ]
  
  ;; are there intersections?
  ;; only check emergency braking situations, ignore safe_dist
  let int_color color-of-nearest-light brake_dist
  ;; fix to allow cars to "inch up" to light
  let inch? false
    
  if (not brake?)
  [
    if (int_color = yellow or int_color = red)
    [
      ;; this should allow cars really close to lights to move up to them
      ifelse ((pcolor = int_color and speed < MAX_ACCELERATION) or 
        (speed > MAX_ACCELERATION and
          ;; don't stop if it is impossible to
          ;; checked by if brake_dist or brake_dist - 1 is the light
          (int_color = red or
            [pcolor = int_color] of patch-ahead (brake_dist) or
            [pcolor = int_color] of patch-ahead (brake_dist - 1)
            )
          )
        )
      [
        ;; checks if next patch is intersection patch, or if on intersection patch but can move a little more
        ifelse (speed = 0 and
          ([pcolor = int_color] of patch-ahead 1 or
            ([pcolor = int_color] of patch-here and
              distance patch-here > 0.1 and
              ;; Bug fix: that check above screws up if we are slightly off patch but going PAST it.  Inching just increases distance
              towards patch-here = heading))
          )
        [
          set inch? true 
        ]
        [
          ;; before we send the signal to stop, let's allow running red lights
          ;; possible if waiting at least x seconds and sense of urgency is high
          if-else (not (running_red? or
              (pcolor = red and
                wait_time > RED_LIGHT_WAIT_MIN and
                sense_of_urgency > law_compliance_threshold and
                cross-traffic-clear? heading)
              )
            )
          [
            set slow_down? true
            set brake? true
          ]
          [
            set running_red? true
            ;; $@LOGVERBOSE
            show (word "*******Running a red light at " ticks)
            ;; $@END
          ]
        ]
      ]
      [
        if (speed <= MAX_ACCELERATION)
        [
          set inch? true
        ]
        if (speed + acceleration > MAX_ACCELERATION)
        [
          set coast? true
        ]
      ]
    ]
  ]
    
  let x item 0 curr_goal
  let y item 1 curr_goal
  let h heading
  ;; check for blocked box
  if ((int_color = green or int_color = yellow or int_color = red) and not inch? and
    (
      ;; only check box-blocked for straight travel
      ;; don't care about blocking the box if sense of urgency > law compliance
      (turn_goal = 0 and sense_of_urgency < law_compliance_threshold and not intersection-clear?) or
      ;; the following avoids going into intersections where cars are still turning left
      ;; this should fix a lot of the wierd jams
      ;; don't care about this when the turners are incapacitated
      ;; don't care about this when running a red light?
      (not running_red? and 
        any? left_turners with
        [
          [column] of patch-here = x and [row] of patch-here = y and
          (heading = (h + 90) mod 360 or heading = (h - 90) mod 360) and
          not incapacitated?
        ]
      ) or
      ;; make sure right turners don't go forward when they can't make the turn
      ;; and don't look too far to opposite lane
      (turn_goal = 90 and right-turn-lane? and
        not right-turn-clear?)
      )
    )
  [
    ;; $@DEBUG
    set $trace $trace + 256
    ;; $@END
    ;; similar inching up code when blocked box
    ifelse (not (speed = 0 and 
          ([pcolor = int_color] of patch-ahead 1 or
            ([pcolor = int_color] of patch-here and
              distance patch-here > 0.1 and
              ;; Bug fix: that check above screws up if we are slightly off patch but going PAST it.  Inching just increases distance
              towards patch-here = heading))
        )
      )
    [
      set slow_down? true
      set brake? true
    ]
    [
      set inch? true 
    ]
  ]
  
  if (not slow_down?)
  [
    ;; check further if we didn't find a light
    if (int_color = black)
    [
      set int_color color-of-nearest-light safe-distance
    ]
    if (speed > SPEED_LIMIT and (int_color = yellow or int_color = red))
    [
      set slow_down? true
    ]
  ]
  
  ifelse (slow_down?)
  [
    let amount (AVG_ACCELERATION)
    if (brake?)
    [
      set amount (MAX_ACCELERATION)
    ]
    brake amount
  ]
  [
    ifelse (not coast?)
    [
      ;; speed up
      if (acceleration < 0)
      [
        ;; start at 0 acceleration or greater
        reset-acceleration
      ]
      accelerate (AVG_ACCELERATION)
    ]
    [
      reset-acceleration
    ]
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Speed Car Ahead
;; @Turtle
;;
;; This will look at the car ahead and send signals to slow down or brake accordingly
;;
;; @param c the car or left turner ahead of this one which we will compare to
;; @report [coast? slow_down? brake?] list with values that update-speed will interpret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report speed-car-ahead [c]
  let slow_down? false
  ;; keep track of how much to slow down
  let brake? false
  let coast? false
  ;; $@DEBUG
  set $trace $trace + 1
  ;; $@END
  
  ;; there are cars ahead of this one, are they slower?
  ifelse (speed + AVG_ACCELERATION > [speed] of c)
  [
    ifelse (speed = 0 and linear-distance c heading > 1 + AVG_ACCELERATION)
    [
      ;; $@DEBUG
      set $trace $trace + 2
      ;; $@END
      ;; speed up a little to inch up
    ]
    [
      ifelse (speed <= MAX_ACCELERATION and linear-distance c heading > 1 + speed + AVG_ACCELERATION)
      [
        ;; $@DEBUG
        set $trace $trace + 4
        ;; $@END
        if (speed != 0 and linear-distance c heading < 2 + speed + AVG_ACCELERATION)
        [
          ;; $@DEBUG
          set $trace $trace + 8
          ;; $@END
          set coast? true
        ]
      ]
      [
        if ((speed + AVG_ACCELERATION) >= [speed] of c and 
          ;; if approx distance between cars 2 steps from now <= Vdiff^2/(2Abrake)
          ;; then car cannot "stop" (match speed) of car ahead in time
          (linear-distance c heading - 2 * (speed - [speed] of c) )
          <= (speed - [speed] of c) * (speed - [speed] of c) / (2 * AVG_ACCELERATION))
        [
          ;; $@DEBUG
          set $trace $trace + 16
          ;; $@END
          set slow_down? true
        ]
        ;; do we need to brake?
        ;; check distance to stop, too close, driving a different direction, and braking
        if (linear-distance c heading - (speed + AVG_ACCELERATION) <= 1 or 
          heading != [heading] of c or
          ;; brake light
          [acceleration] of c = (- MAX_ACCELERATION))
        [
          ;; $@DEBUG
          set $trace $trace + 32
          ;; $@END
          set slow_down? true
          set brake? true
        ]
        
        if (not brake? and 
          speed > [speed] of c and 
          will-cars-collide? self c linear-distance c heading)
        [
          ;; $@DEBUG
          set $trace $trace + 64
          ;; $@END
          set slow_down? true
          set brake? true
        ]
      ]
    ]
  ]
  [
    ;; stop if you are too close, even if they are going faster
    ifelse (linear-distance c heading - (speed + AVG_ACCELERATION) <= 1)
    [
      ;; $@DEBUG
      set $trace $trace + 512
      ;; $@END
      set slow_down? true
      set brake? true
    ]
    [
      ;; check for approximately equal speeds and coast
      if ( abs (speed - [speed] of c) < AVG_ACCELERATION and linear-distance c heading > 1)
      [
        ;; $@DEBUG
        set $trace $trace + 1024
        ;; $@END
        set coast? true
      ]
    ]
  ]
  report (list coast? slow_down? brake?)
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Accelerate
;; @turtle
;;
;; Change the acceleration by some number
;;
;; @param change the positive or negative amount to change acceleration by
;; @post acceleration changes within the range [- MAX_ACCELERATION, MAX_ACCELERATION]
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to accelerate [change]
  set acceleration median (list (- MAX_ACCELERATION) (acceleration + change) MAX_ACCELERATION)
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Brake
;; @turtle
;;
;; Change the acceleration to some negative number
;;
;; @param amount the positive amount to decelerate by (will be negative)
;; @post acceleration changes within the range [- MAX_ACCELERATION, MAX_ACCELERATION]
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to brake [amount]
  set acceleration median (list (- MAX_ACCELERATION) (- amount) 0)
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Reset Acceleration
;; @turtle
;;
;; Change the acceleration to 0
;; Currently a copout for realistic acceleration changes (to brake for instance from any acceleration)
;;
;; @post acceleration is 0
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to reset-acceleration
  set acceleration 0
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Apply Acceleration
;; @turtle
;;
;; Change the speed by the acceleration
;;
;; @post speed changes within the range [0, my_speed_limit]
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to apply-acceleration
  ;; keep speed within bounds
  let temp_speed_limit my_speed_limit
  ;; can increase limit if stressed
  if (sense_of_urgency > speeding_threshold)
  [
    set temp_speed_limit temp_speed_limit + speed_multiplier * sense_of_urgency
  ]
  set speed median (list 0 (speed + acceleration) (temp_speed_limit + (random-float SPEED_FLUX - SPEED_FLUX / 2)))
  
  ;; keep acceleration realistic in case other cars look at it
  if (speed >= temp_speed_limit)
  [
    ;; only looking at upper bound because keeping negative acceleration when
    ;; car stops is like a brake light
    set acceleration 0
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Check for collision
;; @turtle
;;
;; Look at the cars here and very close, see if we have an accident
;;
;; @post crash if there is a collision
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to check-for-collision
  let possible_crash_victims (cars-on neighbors) with
      [heading != [heading] of myself and distance myself < 0.8]
  let i_crashed false
  ask possible_crash_victims
  [
    if (vector-collision myself self)
    [
      ;; $@LOGVERBOSE
      show "******************* Found hidden CRASH"
      show (word "My distance was " distance myself)
      ;; $@END
      crash
      set i_crashed true
    ]
  ]
  if (i_crashed)
  [
    ;; $@LOGVERBOSE
    show "******************* Found hidden CRASH"
    ;; $@END
    crash
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Vector collision
;; @turtle
;;
;; Check if two cars will collide due to their vectors
;;
;; @param c1 First car to check (ideally E/W)
;; @param c2 Second car to check (ideally N/S)
;; @report if there was a crash
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report vector-collision [c1 c2]
  let car1 c1
  let car2 c2
  ;; force car1 to be E/W and car2 to N/S
  if-else ([heading] of car1 = DIR_NORTH or [heading] of car1 = DIR_SOUTH)
  [
    ;; swap
    let temp car1
    set car1 car2
    set car2 temp
    ;; check that car 1 is now correct
    if ([heading] of car1 != DIR_EAST and [heading] of car1 != DIR_WEST)
    [
      ;; parallel, don't care
      report false
    ]
  ]
  [
    ;; check that car 2 is now correct
    if ([heading] of car2 != DIR_NORTH and [heading] of car2 != DIR_SOUTH)
    [
      ;; parallel, don't care
      report false
    ]
  ]
  
  let ew_test false
  ;; figure out which case east/west is
  if-else ([heading] of car1 = DIR_EAST)
  [
    set ew_test ([xcor] of car2 > [xcor] of car1 and [xcor] of car2 < [xcor + speed] of car1)
  ]
  [
    set ew_test ([xcor] of car2 < [xcor] of car1 and [xcor] of car2 > [xcor - speed] of car1)
  ]
  let ns_test false
  ;; figure out north vs south
  if-else ([heading] of car2 = DIR_NORTH)
  [
    set ns_test ([ycor] of car1 > [ycor] of car2 and [ycor] of car1 < [ycor + speed] of car2)
  ]
  [
    set ns_test ([ycor] of car1 < [ycor] of car2 and [ycor] of car1 > [ycor - speed] of car2)
  ]
  
  report ew_test and ns_test
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Find Nearest Car
;; @turtle
;;
;; Find the nearest car ahead for speed control
;; Can look in any direction from calling agent
;;
;; @param max_dist the max distance to look based on how soon the car would need to stop
;; @param dir is the direction to look
;; @param spot the starting patch (may be parallel to calling turtle)
;; @report the nearest car
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report find-nearest-car [max_dist dir spot]
  let nearest <find-nearest-car-without-checking-deadlock> max_dist dir spot
  
  ;; we looked farther so make sure the distance isn't too far
  if (nearest != nobody and 
    (
      ;; bugfix: cars on same patch/spot but different direction get stuck
      [heading] of nearest != heading and 
      ;; break the DEADLOCK: if wait times are the same, just have them ignore each other
      (speed = 0 and wait_time >= [wait_time] of nearest) and
      ;; only an error if that car is waiting on this car
      [<find-nearest-car-without-checking-deadlock> 2 heading patch-here] of nearest = self
      )
    )
  [
    ;; this could be an accident (if that is enabled)
    crash
    ask nearest
    [
      crash
    ]
    ;; remove cars that are out of range or fix deadlock
    set nearest nobody
  ]
  
  report nearest
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Find Nearest Car Without Checking Deadlock
;; @turtle
;;
;; @note THIS DOES NOT CHECK IF THE "NEAREST" CAR IS ON THE SAME PATCH AND DEADLOCKED
;; DO NOT CALL THIS FUNCTION DIRECTLY
;; I am using functions named like <this> for "private" things
;;
;; Find the nearest car ahead for speed control
;; Can look in any direction from calling agent
;;
;; @param max_dist the max distance to look based on how soon the car would need to stop
;; @param dir is the direction to look
;; @param spot the starting patch (may be parallel to calling turtle)
;; @report the nearest car
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report <find-nearest-car-without-checking-deadlock> [max_dist dir spot]
  if (max_dist < 1)
  [
    set max_dist 1
  ]
  let near no-turtles
  let i 1
  let next no-patches
  let prev no-patches
  ask spot
  [
    set next patch-at-heading-and-distance dir 1
    set prev patch-at-heading-and-distance dir (- 1)
  ]
  
  ;; get cars on the same patch first
  if (any? other cars-on spot)
  [
    let d distance next
    set near (other cars-on spot) with [distance next < d]
  ]
  ;; check in patch context based on spot, not turtle
  ask spot
  [
    ;; loop to find cars up to a max distance (don't waste my time)
    while [i <= max_dist + 1 and not any? near]
    [
      set near (cars-on patch-at-heading-and-distance dir i)
      set i i + 1
    ]
  ]
  
  ;; pick the actual nearest out of agentset
  let nearest (min-one-of near [distance prev])
  
  ;; we looked farther so make sure the distance isn't too far
  if (nearest != nobody and 
    (linear-distance nearest dir > max_dist)
    )
  [
    ;; remove cars that are out of range
    set nearest nobody
  ]
  
  ;; emergency is non-toroidal, fix that
  if (alarm? and nearest != nobody and 
    (
      (dir = DIR_EAST and xcor > [xcor] of nearest) or
      (dir = DIR_WEST and xcor < [xcor] of nearest) or
      (dir = DIR_NORTH and ycor > [ycor] of nearest) or
      (dir = DIR_SOUTH and ycor < [ycor] of nearest)
      )
    )
  [
    ;; the nearest is on the opposite side
    set nearest nobody
  ]
  
  report nearest
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Find Nearest Left Turner
;; @turtle
;;
;; Find the nearest left_turner ahead for speed control
;; Can look in any direction from calling agent
;;
;; @param max_dist the max distance to look based on how soon the car would need to stop
;; @param dir is the direction to look
;; @param spot the starting patch (may be parallel to calling turtle)
;; @report the nearest car
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report find-nearest-left-turner [max_dist dir spot]
  if (max_dist < 1)
  [
    set max_dist 1
  ]
  let near no-turtles
  let i 1
  let next no-patches
  let prev no-patches
  ask spot
  [
    set next patch-at-heading-and-distance dir 1
    set prev patch-at-heading-and-distance dir (- 1)
  ]
  
  ;; get cars on the same patch first
  if (any? other left_turners-on spot)
  [
    let d distance next
    set near (other left_turners-on spot) with [distance next < d]
  ]
  ;; check in patch context based on spot, not turtle
  ask spot
  [
    ;; loop to find cars up to a max distance (don't waste my time)
    while [i <= max_dist + 1 and not any? near]
    [
      set near (left_turners-on patch-at-heading-and-distance dir i)
      set i i + 1
    ]
  ]
  
  ;; pick the actual nearest out of agentset
  let nearest (min-one-of near [distance prev])
  
  ;; we looked farther so make sure the distance isn't too far
  if (nearest != nobody and 
    (linear-distance nearest dir > max_dist or
    ;; bugfix: cars on same patch/spot but different direction get stuck
      ([heading] of nearest != heading and 
        ([speed] of nearest < speed or 
          (speed = 0 and wait_time > [wait_time] of nearest)
          )
        )
      )
    )
  [
    ;; remove cars that are out of range
    set nearest nobody
  ]
  
  ;; emergency is non-toroidal east-west, fix that
  if (alarm? and nearest != nobody and 
    (
      (heading = DIR_EAST and xcor > [xcor] of nearest) or
      (heading = DIR_WEST and xcor < [xcor] of nearest) or
      (heading = DIR_NORTH and ycor > [ycor] of nearest) or
      (heading = DIR_SOUTH and ycor < [ycor] of nearest)
      )
    )
  [
    ;; the nearest is on the opposite side
    set nearest nobody
  ]
  
  report nearest
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Color of Nearest Light
;; @turtle
;;
;; Find the nearest intersection ahead and report its state
;; @param max_dist the max distance to look based on how soon the car would need to stop
;; @report the color of the nearest intersection or black signalling none
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report color-of-nearest-light [max_dist]
  ;; require distance to be >= 1
  if (max_dist < 1)
  [
    set max_dist 1
  ]
  let i 0
  while [i <= max_dist]
  [
    let col ([pcolor] of patch-ahead i)
    if (col = green or col = yellow or col = red)
    [
      ;; it's a light
      report col
    ]
    set i i + 1
  ]
  ;; no intersection case
  report black
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Safe Distance
;; @turtle
;;
;; Find the safe distance in which the car can slow down if need be
;;
;; @report the distance in which the car could safely decelerate to a stop
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report safe-distance
  ;; remember: vf = vi + a * t
  ;;           d = vf * t - 1/2 a * t^2
  ;;           d = vi * t + 1/2 a * t^2
  ;;           vf = vi + a * t
  ;;           vf^2 = vi^2 + 2 * a * d
  ;; time to reach speed = 0 at light brake
  let t ceiling ((speed) / AVG_ACCELERATION)
  ;; distance to stop
  let dist (speed * t - AVG_ACCELERATION * t * t / 2)
  
  ;; look one beyond that because we don't want to stop on top of cars
  report ceiling dist + 1
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Brake Distance
;; @turtle
;;
;; Find the distance in which the car can come to a stop 
;;
;; @report the distance in which the car could decelerate to a stop while braking hard
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report brake-distance
  ;; remember: vf = vi + a * t
  ;;           d = vf * t - 1/2 a * t^2
  ;;           d = vi * t + 1/2 a * t^2
  ;;           vf = vi + a * t
  ;;           vf^2 = vi^2 + 2 * a * d
  ;; time to reach speed = 0 at full brake
  let t ceiling (speed / MAX_ACCELERATION)
  ;; distance to stop
  let dist (speed * t - MAX_ACCELERATION * t * t / 2)
  
  ;; look one beyond that because we don't want to stop on top of cars
  report ceiling dist + 1
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Linear Distance
;; @turtle
;;
;; Find the distance from one car to the next assuming they are in the same lane
;; Just compare one coordinate based on heading
;;
;; @param c turtle to check distance with
;; @param dir the direction that the calling car is looking (default should be heading)
;; @pre c is in front of calling turtle (based on dir)
;; @report the distance to the next car regardless of the lane
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report linear-distance [c dir]
  if (dir = DIR_WEST)
  [
    if ([xcor] of c > xcor)
    [
      ;; toroidal edge
      report (max-pxcor - [xcor] of c) + (xcor - min-pxcor) + 1
    ]
    report xcor - [xcor] of c
  ]
  if (dir = DIR_EAST)
  [
    if ([xcor] of c < xcor)
    [
      ;; toroidal edge
      report ([xcor] of c - min-pxcor) + (max-pxcor - xcor) + 1
    ]
    report [xcor] of c - xcor
  ]
  if (dir = DIR_SOUTH)
  [
    if ([ycor] of c > ycor)
    [
      ;; toroidal edge
      report (max-pycor - [ycor] of c) + (ycor - min-pycor) + 1
    ]
    report ycor - [ycor] of c
  ]
  if (dir = DIR_NORTH)
  [
    if ([ycor] of c < ycor)
    [
      ;; toroidal edge
      report ([ycor] of c - min-pycor) + (max-pycor - ycor) + 1
    ]
    report [ycor] of c - ycor
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Can Turn Right on Red
;; @turtle
;;
;; Check if a right on red would work
;; @pre turtle is turning right, is in the right turn lane and is on a yellow or red patch
;; @report true if right on red is allowed
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report can-turn-right-on-red?
  let angle heading
  ;; p is patch ahead
  let p patch-ahead 1
  ;; now p is patch ahead and right
  ask p 
  [
    set p patch-at-heading-and-distance (angle + 90) 1
  ]
  let clear? false
  ask p
  [
    ;; right on red routine
    ;; BUG fix: make sure it's not a one-way road or cars will go STRAIGHT on red, whoops
    ;; Fix: make sure that traffic is clear straight ahead and to the right, not just to the left
    if (member? ((angle + 90) mod 360) flow_dir and
      not any-cars-here-to-dist? (angle - 90) RIGHT_RED_DIST)
    [
      ;; move into intersection so that right turn will happen
      set clear? true
    ]
  ]
  report clear?
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Right on Red
;; @turtle
;;
;; Perform a right turn at a red light
;; @pre turtle is turning right, is in the right turn lane and is on a yellow or red patch
;; @post if possible, turtle moves to patch ahead and turns right
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to right-on-red
  if (can-turn-right-on-red?)
  [
    move-to patch-ahead 1
    turn-right
    ;; even though speed is still 0, it is no longer waiting
    set wait_time 0
    ;; new intersection, need to update goal or we will try to turn again
    update-goal false
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Any Cars Here To Dist?
;; @Patch
;;
;; Looks dist patches ahead at angle to see if there are any cars
;;
;; @param angle the angle to look from this patch
;; @param dist the distance to look
;; @report true if there are any cars on this patch or within this distance inclusive
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report any-cars-here-to-dist? [angle dist]
  ;; positive distances only
  if dist < 1
  [
    set dist 1
  ]
  let i 0
  while [i <= dist]
  [
    if (any? (cars-on patch-at-heading-and-distance angle i))
    [
      if (any? ((cars-on patch-at-heading-and-distance angle i) with [incapacitated?]))
      [
        ;; if they are incapacitated, who cares?
        report false
      ]
      report true
    ]
    set i i + 1
  ]
  
  ;; default
  report false
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Intersection Ahead?
;; @Turtle
;;
;; Checks to see if there is an intersection "dist" patches ahead of this turtle
;; Checks to make sure that intersection is part of that turtle's curr_goal so that
;; turtles ignore intersections that they would not really want to turn at
;;
;; @param dist the distance ahead to look (forced to be >= 1)
;; @report boolean whether or not there is an intersection aheah
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report intersection-ahead? [dist]
  ;; require distance to be >= 1
  if (dist < 1)
  [
    set dist 1
  ]
  let i 1
  while [i <= dist]
  [
    if ([intersection?] of patch-ahead i and
        (([row] of patch-ahead i = item 1 curr_goal) and 
        ([column] of patch-ahead i = item 0 curr_goal))
      )
    [
      report true
    ]
    set i i + 1
  ]
  
  report false
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Cross Traffic Clear?
;; @Turtle
;;
;; Uses a search to see if the cross traffic is clear for long enough to run a red light
;; This can be influenced by sense_of_urgnecy, that is cars may not be as careful as that increases
;;
;; @param my_dir The direction I am going, need to look in the perpendicular directions
;; @pre car is waiting at red light (on red light patch)
;; @report true if the cross traffic is clear and the car can run the light without an accident
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report cross-traffic-clear? [my_dir]
  let p patch-ahead 1
  let curr_lane 1
  ;; chance of not looking both ways
  let urgency_mistake ifelse-value (sense_of_urgency > tunnel_vision_threshold) [sense_of_urgency * STRESS_ERROR_RATE] [0]
  
  ;; loop through the intersection to the other side
  while [[intersection?] of p]
  [
    ;; look in the direction of traffic
    if (any? cars-on p)
    [
      ;; will never miss the obvious
      report false
    ]
    ;; want to search in perpendicular direction to car (reverse of flow)
    let search_dir (180 + first (filter [? != my_dir] ([flow_dir] of p))) mod 360
    let lane p
    while [[intersection?] of lane]
    [
      ask lane
      [
        set lane patch-at-heading-and-distance search_dir 1
      ]
      if (any? cars-on lane)
      [
        ;; @todo Would they always see guys in the intersection?
        report false
      ]
    
    ]
    ;; look an additional curr_lane + 2 spaces
    let i 0
    while [i < curr_lane + 2 and i < MAX_CROSS_CHECK_DIST]
    [
      ask lane
      [
        set lane patch-at-heading-and-distance search_dir 1
      ]
      if (any? cars-on lane)
      [
        ;; chance of not looking out of the intersection
        if (random-float 1 > urgency_mistake)
        [
          report false
        ]
      ]
      
      set i i + 1
    
    ]

    ;; increment to next lane
    set curr_lane curr_lane + 1
    set p patch-ahead curr_lane
  ]
  
  ;; $@DEBUG
  print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ Good to go!"
  ;; $@END
  report true
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Intersection Clear?
;; @Turtle
;;
;; Uses a search ahead to see if the intersection is clear and the car can safely exit
;; Goal to avoid blocking the box
;;
;; @pre car is approaching a light
;; @report true if the intersection is clear and false if it would create a blocked box
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report intersection-clear?
  let h heading
  let i 0
  let p patch-ahead i
  ;; number of empty patches past intersection
  let empty_spaces 0
  ;; number of cars in between to add to the number to look ahead
  ;; bug: don't count huge number of cars?
  ;; fix: ignore the cars here incase we have issues with too many in one spot
  let car_count 1 ;+ count cars-on p
  
  ;; loop up to the intersection
  while [not [intersection?] of p and not EXCEPTION?]
  [
    ;; on the road, skip ahead to intersection
    set i i + 1
    set p patch-ahead i
    
    ;; count cars in the intersection
    set car_count car_count + count cars-on p
    if (i > 100)
    [
      ;; this should NEVER happen
      carefully 
      [
        set error_text (word "BROKEN INFINITE LOOP!!!!!!!!!! at " ticks ", called by " myself)
      ]
      [
        set error_text (word "BROKEN INFINITE LOOP!!!!!!!!!! at " ticks ", called by self") ;; no calling agent myself
      ]
      set error_text (word error_text "\n    Heading: " heading ", xcor:" xcor ", ycor:" ycor)
      show error_text
      set EXCEPTION? true
      log-error
      report false ;; dummy report value, just get out quickly
      ;stop
    ]
  ]
  
  if (any? (cars-on p) with [
    speed = 0 or
    heading != h]
    )
  [
    report false
  ]
  
  ;; loop through intersection to check all patches
  while [[intersection?] of p]
  [
    set i i + 1
    set p patch-ahead i
    
    ;; when we get past the interseciton, back up and go to next loop
    if-else (not [intersection?] of p)
    [
      set i i - 1
    ]
    [
    
      ;; check for wrong direciton, stopped, and cars waiting in long lines
      if (any? (cars-on p) with [
        speed = 0 or
        heading != h or
        queue > BOX_BLOCK_QUEUE_LIMIT]
        )
      [
        report false
      ]
      
      ;; count cars in the intersection
      set car_count car_count + count cars-on p
    ]
  ]
  
  ;; look beyond for the number of cars ahead plus one
  while [empty_spaces < car_count]
  [
    set i i + 1
    set p patch-ahead i
    
    ;; check for wrong direciton, stoped, and cars waiting in long lines
    if (any? (cars-on p) with [
      speed = 0 or
      heading != h or
      queue > BOX_BLOCK_QUEUE_LIMIT]
    )
    [
      report false
    ]
    if (not any? cars-on p)
    [
      set empty_spaces empty_spaces + 1
    ]
    
    ;; this means we have looked and there are not enough empty spaces
    if ([intersection?] of p)
    [
      report false
    ]
  ]
  
  report true
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Turn Distance
;; @Turtle
;;
;; Used for right turners to look ahead a certain distance based on their speed
;; so that they can slow down at the intersection
;;
;; @report how far ahead to check based on how long it will take
;; to slow down to the TURN_SPEED
;;
;; @pre speed > TURN_SPEED
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report turn-distance
  ;report ((speed - TURN_SPEED) / AVG_ACCELERATION + 2)
  ;; remember: vf = vi + a * t
  ;;           d = vf * t - 1/2 a * t^2
  ;;           d = vi * t + 1/2 a * t^2
  ;;           vf = vi + a * t
  ;;           vf^2 = vi^2 + 2 * a * d
  let t ceiling ((speed - TURN_SPEED) / AVG_ACCELERATION)
  let dist (speed * t - AVG_ACCELERATION * t * t / 2)
  
  report ceiling dist
end

;################################################################################
;; Evacuation Respawning
;################################################################################

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Sound Alarm
;; @Observer
;; @ask turtles
;;
;; Sets the emergency "alarm?" flag to true and tries to tell all of the turtles
;; Only 50% will "hear" alarm
;; Also begins to change traffic lights to new timings by reading new data
;;
;; @post alarm? set to true
;; @post PERCENT_HEAR_ALARM of the turtles will attempt to evacuate (evac? true)
;; and remove previous goals
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to sound-alarm
  print (word ticks ": Sounding Alarm")
  set alarm? true
  read-new-signal-timing
  
  ;; no automatic 50% "hear alarm" if radios are used
  if (not radios?)
  [
    ask (turtle-set cars left_turners)
    [
      ;; random chance of "hearing" alarm
      if (random-float 1 < PERCENT_HEAR_ALARM)
      [
        ;; $@DEBUG
        set $goal_change $goal_change + 1
        ;; $@END
        set leave? false
        set evac? true
        set evac_goal evacuation-direction
        set local_goal []
        update-goal true ;; update no matter where they are and don't change the current goal
      ]
    ]
  ]
  
  ;; cars off map to east or west are evacuated
  ask off_map_cars with [heading = DIR_WEST or heading = DIR_EAST]
  [
    ;; evacuated
    set cars_safe cars_safe + 1
    set breed objects
    hide-turtle
    if (count objects > (sum [car_amount] of garages))
    [
      die
    ]
  ]
  
  ;; new cars to spawn at an emergency are created now
;  create-danger_cars EXTRA_RESPAWN
;  [
;    hide-me true
;    put-on-danger-road
;    setup-car
;  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Check Bounds
;; @Turtle
;;
;; Cars leaving any side of the map are stored in an agent-set and later "respawn
;; when the alarm is set, if cars driver off of the east or west side, they will be evacuated
;;
;; @pre speed is no greater than 2 (hard-coded)
;; @post turtle may die if it leaves map and this will update
;; the cars_safe and cars_off_map counters
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to check-bounds
  if (
    (heading = DIR_EAST and 
      ((pxcor = max-pxcor) or
        (speed > 0 and ([pxcor] of patch-ahead 1) = max-pxcor) or
        (speed > 1 and ([pxcor] of patch-ahead 2) = max-pxcor)
        )
      ) or
    (heading = DIR_WEST and 
      ((pxcor = min-pxcor) or
        (speed > 0 and ([pxcor] of patch-ahead 1) = min-pxcor) or
        (speed > 1 and ([pxcor] of patch-ahead 2) = min-pxcor)
        )
      ) or
    (heading = DIR_NORTH and 
      ((pycor = max-pycor) or
        (speed > 0 and ([pycor] of patch-ahead 1) = max-pycor) or
        (speed > 1 and ([pycor] of patch-ahead 2) = max-pycor)
        )
      ) or
    (heading = DIR_SOUTH and 
      ((pycor = min-pycor) or
        (speed > 0 and ([pycor] of patch-ahead 1) = min-pycor) or
        (speed > 1 and ([pycor] of patch-ahead 2) = min-pycor)
        )
      )
    )
  [
    ifelse (alarm? and (heading = DIR_EAST or heading = DIR_WEST))
    [
      ;; evacuated
      set cars_safe cars_safe + 1
      recycle
    ]
    [
      ;;store car for later
      set cars_off_map cars_off_map + 1
      hide-me
    ]
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Respawn Cars
;; @Observer
;;
;; Respawn cars back on to the map after they have driven off
;; Happens every time cars are ready to respawn based on timer and
;; the cars will try to evacuate 80% of the time
;; Uses cars of off_map_car breed
;;
;; @post respawns as many cars as possible on the west side of the map and
;; decrements the cars_off_map
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to respawn-cars
  ;; loop while there are cars to respawn and space to put them on
  ;; must be done for each direction
  
;  ;; east/west do not spawn during alarm
;  if (not alarm?)
;  [
;    ;; West
;    while [(any? off_map_cars with [time <= 0 and heading = DIR_WEST]) and
;      ;; empty road to place on
;      any? roads with
;      [
;        pxcor = min-pxcor and
;        not (any? cars-on self) and
;        member? DIR_EAST flow_dir
;      ]
;    ]
;    [
;      ask one-of off_map_cars with [time <= 0 and heading = DIR_WEST]
;      [
;        ;; place on appropriate road
;        put-on-entrance
;        
;        ;; make visible
;        show-me
;        
;        ;; do some reseting so that cars don't run into one another
;        set speed 0
;        set acceleration 0
;        
;        ;; figure out goals! (more likely to evacuate based on PERCENT_HEAR_AFTER)
;        find-new-goal
;        find-first-goal
;        update-goal true
;      ]
;      set cars_off_map cars_off_map - 1
;    ]
;    
;    ;; East
;    while [any? off_map_cars with [time <= 0 and heading = DIR_EAST] and
;      ;; empty road to place on
;      any? roads with
;      [
;        pxcor = max-pxcor and
;        not (any? cars-on self) and
;        member? DIR_WEST flow_dir
;      ]
;    ]
;    [
;      ask one-of off_map_cars with [time <= 0 and heading = DIR_EAST]
;      [
;        ;; place on appropriate road
;        put-on-entrance
;        
;        ;; make visible
;        show-me
;        
;        ;; do some reseting so that cars don't run into one another
;        set speed 0
;        set acceleration 0
;        
;        ;; figure out goals! (more likely to evacuate based on PERCENT_HEAR_AFTER)
;        find-new-goal
;        find-first-goal
;        update-goal true
;      ]
;      set cars_off_map cars_off_map - 1
;    ]
;  ]
  
  ;; North
  while [any? off_map_cars with [time <= 0 and heading = DIR_NORTH] and
    ;; empty road to place on
    any? roads with
    [
      pycor = max-pycor and
      not (any? cars-on self) and
      member? DIR_SOUTH flow_dir
    ]
    ]
  [
    ask one-of off_map_cars with [time <= 0 and heading = DIR_NORTH]
    [
      ;; place on appropriate road
      put-on-entrance
      
      ;; make visible
      show-me
      
      ;; do some reseting so that cars don't run into one another
      set speed 0
      set acceleration 0
      
      ;; figure out goals! (more likely to evacuate based on PERCENT_HEAR_AFTER)
      find-new-goal
      find-first-goal
      update-goal true
    ]
    set cars_off_map cars_off_map - 1
  ]
  
  ;; South
  while [any? off_map_cars with [time <= 0 and heading = DIR_SOUTH] and
    ;; empty road to place on
    any? roads with
    [
      pycor = min-pycor and
      not (any? cars-on self) and
      member? DIR_NORTH flow_dir
    ]
    ]
  [
    ask one-of off_map_cars with [time <= 0 and heading = DIR_SOUTH]
    [
      ;; place on appropriate road
      put-on-entrance
      
      ;; make visible
      show-me
      
      ;; do some reseting so that cars don't run into one another
      set speed 0
      set acceleration 0
      
      ;; figure out goals! (more likely to evacuate based on PERCENT_HEAR_AFTER)
      find-new-goal
      find-first-goal
      update-goal true
    ]
    set cars_off_map cars_off_map - 1
  ]
  
  ;; garages
  spawn-garage-cars
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Spawn Garage Cars
;; @Observer
;; @ask garages
;;
;; Create cars of the garage_car breed at garages where there is space available
;;
;; @pre alarm? = true
;; @post respawns cars at every garage with cars remaining and space to create them
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to spawn-garage-cars
  ask garages
  [
    if (car_amount > 0 and not any? garage_cars-on patch-ahead 1)
    [
      set car_amount car_amount - 1
      set label car_amount
      
      ;; get a new car and set it up
      let c new-car-hatch
      let space patch-ahead 1
      ask c
      [
        move-to space
        show-turtle
        set breed garage_cars
        setup-car
        ;; car must make a turn
        set turn_goal [turn_dir] of myself
        find-queue-garage
        ;; update this cars exposure to the level of the garage
        set exposure [exposure] of myself
      ]
    ]
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Hide Me
;; @Turtle
;;
;; Make this car hidden so that it is off the map
;; Separate cars by their headings
;;
;; @post car is invisible and ignored, breed = off_map_car
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to hide-me
  set breed off_map_cars
  hide-turtle
  ;; make cars wait random time
  set time (random (TIME_TO_RESPAWN_MAX - TIME_TO_RESPAWN)) + TIME_TO_RESPAWN
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Show Me
;; @Turtle
;;
;; Make this hidden car visible so it can drive back onto the map
;;
;; @post car is visible, breed = cars
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to show-me
  set breed cars
  show-turtle
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; New Car
;; @Observer
;;
;; Get a new car from the pool of used objects or create one
;;
;; @report car agent that is either reused or new
;; @post car is visible, calls put-on-entrance, setup-car
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report new-car
  let c nobody
  ;; get the new car
  ifelse (any? objects)
  [
    set c one-of objects
  ]
  [
    ;; create extra cars but only use one of them this time
    create-objects 5
    [
      hide-turtle
      set c self
    ]
  ]
  
  report c
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; New Car Hatch
;; @Turtle
;;
;; Get a new car from the pool of used objects or create one
;; This is used when in turtle context
;;
;; @report car agent that is either reused or new
;; @post car is visible, calls put-on-entrance, setup-car
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report new-car-hatch
  let c nobody
  ;; get the new car
  ifelse (any? objects)
  [
    set c one-of objects
  ]
  [
    ;; create extra cars but only use one of them this time
    hatch-objects 5
    [
      set size 1
      set label ""
      hide-turtle
      set c self
    ]
  ]
  
  report c
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Recycle
;; @Turtle
;;
;; Have this turtle either die or throw it in the pool of reusable turtles
;;
;; @pre  car is done evacuated or is some temporary car that we can remove now
;; @post car is no longer usable
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to recycle
  set breed objects
  hide-turtle
  if (count objects > (sum [car_amount] of garages))
  [
    die
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Put On Entrance
;; @Turtle
;;
;; Place respawn cars on the west side of the map heading east
;;
;; @pre  turtle _must_ be an off_map_car (call show-me after)
;; @post respawned turtles will be placed on the edge of the map with correct heading
;; @post must call show-me
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to put-on-entrance
  ifelse (heading = DIR_NORTH)
  [
    move-to one-of roads with
    [
      pycor = max-pycor and
      (not any? cars-on self) and
      member? DIR_SOUTH flow_dir
    ]
  ]
  [
    ifelse (heading = DIR_SOUTH)
    [
      move-to one-of roads with
      [
        pycor = min-pycor and
        (not any? cars-on self) and
        member? DIR_NORTH flow_dir
      ] 
    ]
    [
      ifelse (HEADING = DIR_EAST)
      [
        move-to one-of roads with
        [
          pxcor = max-pxcor and
          (not any? cars-on self) and
          member? DIR_WEST flow_dir
        ]
      ]
      [
        ;; west
        move-to one-of roads with
        [
          pxcor = min-pxcor and
          (not any? cars-on self) and
          member? DIR_EAST flow_dir
        ]
      ]
    ]
  ]
  
  
  set heading [first flow_dir] of patch-here
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Find Queue Garage
;; @Garage_cars
;;
;; Determine the turtle's position in line with other turtles
;; This "queue" number is set to 0 or 1 more than the turtle ahead
;;
;; @pre  turtles should be sorted by queue number so that the turtle
;; ahead of this one has the correct queue number
;; @post this turtle should have an accurate number of where it is in line (queue)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to find-queue-garage
  let car_ahead garage_cars-on patch-ahead 1
  ifelse (any? car_ahead)
  [
    set queue ([queue] of min-one-of car_ahead [queue]) + 1
  ]
  [
    set queue 0
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Move Garage Car
;; @Garage_cars
;;
;; Move this car up the driveway to the red light then try to turn when traffic is clear
;;
;; @pre  this is a garage_car on a driveway
;; @post this garage_car will keep moving forward as possible and then merge onto the road as a normal car
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to move-garage-car
  find-queue-garage
  ifelse (pcolor != red and not any? garage_cars-on patch-ahead 1)
  [
    ;; move forward
    move-to patch-ahead 1
    set wait_time 0
  ]
  [
    if (pcolor = red and not any? cars-on patch-ahead 1)
    [
      ;; attempt a turn
      turn-garage-car
    ]
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Turn Garage Car
;; @Garage_cars
;;
;; Have this car attempt to turn onto the road
;;
;; @pre  this is a garage_car on a driveway at the red light
;; @post this garage_car will move onto the road and turn if possible
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to turn-garage-car
  let angle heading
  ;; p is patch ahead
  let p patch-ahead 1
  let clear? false
  ask p
  [
    ;; turn on red routine
    if (member? ((angle + [turn_goal] of myself) mod 360) flow_dir and
      not any-cars-here-to-dist? (angle - [turn_goal] of myself) RIGHT_RED_DIST)
    [
      ;; move into intersection so that right turn will happen
      set clear? true
    ]
  ]
  if (clear?)
  [
    move-to patch-ahead 1
    ;; make the turn
    set heading (angle + turn_goal)
    ;; make it a normal car
    set breed cars
    ;; even though speed is still 0, it is no longer waiting
    set wait_time 0
    ;; set goals now that we are on the road
    find-new-goal
    find-first-goal
    update-goal true
    ;; bugfix for left_turners stacking up in one space
    if (turn_goal = -90 and any? left_turners-on patch-here)
    [
      set turn_goal 0
    ]
  ]
end

;################################################################################
;; Awareness of emergency and Knowledge of city map
;################################################################################

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Update Car Sign Vision
;; @Signs
;;
;; Mark which cars can see a sign based on each sign's vision cone
;;
;; @pre  emergency is occuring, signs are setup
;; @post cars see_sign may increment
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to update-car-sign-vision
  ask turtles-on vision_patches
  [
    ;; all cars driving toward sign in its cone of vision will see it
    if (heading = (([heading] of myself) + 180) mod 360)
    [
      set see_sign see_sign + 1
    ]
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Update Awareness Knowledge
;; @Turtle
;;
;; Update the awareness and knowledge levels of turtles.
;; Radio can turn on or off with a small probability
;;
;; @pre  emergency is occuring
;; @post radio? may toggle, awareness and knowledge may increase
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to update-awareness-knowledge
  ;; AEGL-1 and pedestrian AEGL-2 effects
  ;; small percentage may turn on radio when smelling gas or seeing AEGL2 pedestrians
  ;; double percentages if awareness above threshold (0.25)
  let my_percent_aegl1_radio ifelse-value (awareness > AWARENESS_THRESHOLD_LOW / 2) [PERCENT_AEGL1_RADIO * 2] [PERCENT_AEGL1_RADIO]
  let my_percent_aegl2ped_radio ifelse-value (awareness > AWARENESS_THRESHOLD_LOW / 2) [PERCENT_AEGL2PED_RADIO * 2] [PERCENT_AEGL2PED_RADIO]
  if ((aeglevel = 1 and random-float 1 < my_percent_aegl1_radio and radios?) or
    (pedestrians_AEGL2? and random-float 1 < my_percent_aegl2ped_radio and radios?)
    )
  [
    set radio? true
  ]
  
  ;; some percentage of AEGL1 drivers will become "aware" of issue even without radio
  ;; only become up to half aware without radio
  ;; ditto for cars witnessing AEGL2 pedestrians
  if (
    (
      (aeglevel = 1 and random-float 1 < PERCENT_AEGL1_AWARE) or
      (pedestrians_AEGL2? and random-float 1 < PERCENT_AEGL2PED_AWARE)
      ) and
    (awareness < MAX_AWARENESS / 2 or (radio? and awareness < MAX_AWARENESS)) and 
    radios?
    )
  [
    set awareness min (list MAX_AWARENESS (awareness + random-normal AWARENESS_DELTA_AVERAGE AWARENESS_DELTA_SD))
  ]
  
  ;; update radio occasionally
  if (radios?)
  [
    let my_percent_radio_change ifelse-value (awareness > AWARENESS_THRESHOLD_LOW / 2) [PERCENT_RADIO_CHANGE * 2] [PERCENT_RADIO_CHANGE]
    ;; Do not turn OFF radio IF awareness above threshold OR sign told them to turn it on
    if (
      (not radio? or (radio? and awareness < AWARENESS_THRESHOLD_LOW and see_sign <= SIGN_TIME)) and
      random-float 1 < my_percent_radio_change)
    [
      set radio? not radio?
    ]
    
    ;; signs instruct drivers
    if (not radio? and see_sign > SIGN_TIME and signs?)
    [
      set radio? true
    ]
    
    ;; awareness increases as radio is on up to a max
    if (radio? and awareness < MAX_AWARENESS)
    [
      set awareness min (list MAX_AWARENESS (awareness + random-normal AWARENESS_DELTA_AVERAGE AWARENESS_DELTA_SD))
    ]
  ]
  
  ;; knowledge increases as cars drive past an intersection
  if (block_time = 1 and knowledge < MAX_KNOW and knowledge?)
  [
    ;; this means they just left an intersection
    set knowledge min (list MAX_KNOW (knowledge + KNOW_DELTA_INTERSECTION))
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Update Sense of Urgency
;; @Turtle
;;
;; Update the sense of urgency level of the turtle if enabled
;; If "stress" is disabled, turtles will always have 0 sense of urgency (assumption in other calculations)
;;
;; @pre  emergency is occuring
;; @post sense of urgency may change
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to update-sense-of-urgency
  if (stress?)
  [
    let stressors? false
    
    ;; increase at approximately 20% per minute if aware to some degree
    if (awareness > AWARENESS_THRESHOLD_LOW)
    [
      ;; awareness factors in so that 50% awareness is standard, more or less means more or less urgency
      set sense_of_urgency sense_of_urgency + (STRESS_AWARENESS_INC * awareness / 2)
      set stressors? true
    ]
    
    ;; if they can smell the gas, they will increase in sense of urgency
    if (aeglevel >= 1)
    [
      set sense_of_urgency sense_of_urgency + STRESS_AEGL_INC
      set stressors? true
    ]
    
    ;; if they see pedestrians passing out, they will increase in sense of urgency
    if (pedestrians_AEGL2?)
    [
      set sense_of_urgency sense_of_urgency + STRESS_PEDESTRIANS_INC
      set stressors? true
    ]
    
    if-else (wait_time >= STRESS_WAIT_THRESHOLD)
    [
      set sense_of_urgency sense_of_urgency + STRESS_WAIT_INC
      set stressors? true
    ]
    [
      if (wait_time = 0)
      [
        set sense_of_urgency sense_of_urgency - STRESS_WAIT_DEC
      ]
    ]
    
    if ( not stressors? )
    [
      set sense_of_urgency sense_of_urgency - STRESS_FADE
    ]
    
    ;; sense of urgency is a value from 0 to 1
    set sense_of_urgency median (list 0 sense_of_urgency 1)
  ]
end

;################################################################################
;; Chemical Spill During Emergency
;################################################################################

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Update Plume
;; @Observer
;; @ask: patches
;;
;; Update the concentration level of each patch based on the time
;; Keep track of pedestrian exposure based on air exchanges
;;
;; @pre  emergency is occuring, ticks are at an update point (60 ticks)
;; @post concentration at every patch changes
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to update-plume
  ;; update concentration levels using data at specified intervals
  if ((ticks - PLUME_START) mod PLUME_UPDATE = 0)
  [
    ;; don't do anything if there is no more data
    if (plume? and any? patches with [not empty? conc_list])
    [
      ask patches
      [
        if-else (empty? conc_list)
        [
          set concentration -1
        ]
        [
          ;; pop next time off of queue
          set concentration first conc_list
          set conc_list but-first conc_list
        ]
      ]
      
      ;; now the "key" patches has their concentration, so we must find intermediate values
      
      ;; lets look at the rows with known data and interpolate the intermediate values
      let rows []
      let cols []
      ask conc_points
      [
        set rows lput pycor rows
        set cols lput pxcor cols
      ]
      set rows remove-duplicates rows
      set cols remove-duplicates cols
      set rows sort rows
      set cols sort cols
      
      foreach rows
      [
        ask patches with [concentration = -1 and pycor = ?]
        [
          let a nobody
          let b nobody
          ;; find nearest known to left and right
          foreach cols
          [
            if (b = nobody and ? > pxcor)
            [
              ;; the patch to the right
              set b patch ? pycor
              ;; back up one spot for the patch to the left
              if ((position ? cols) > 0)
              [
                set a patch (item ((position ? cols) - 1) cols) pycor
              ]
              ;stop
            ]
          ]
          ;; out of range
          if (b = nobody)
          [
            set b patch max-pxcor pycor
            ask b
            [
              set concentration 0
            ]
            set a patch (last cols) pycor
          ]
          if (a = nobody)
          [
            set a patch min-pxcor pycor
            ask a
            [
              set concentration 0
            ]
          ]
          let d_ab abs ([pxcor] of b - [pxcor] of a)
          let d_a abs (pxcor - [pxcor] of a)
          let d_b abs ([pxcor] of b - pxcor)
          set concentration ([concentration] of a * d_b + [concentration] of b * d_a) / d_ab
        ]
      ]
      ask patches with [concentration = -1]
      [
        let a nobody
        let b nobody
        ;; find nearest known above and below
        foreach rows
        [
          if (b = nobody and ? > pycor)
          [
            ;; the patch above
            set b patch pxcor ?
            ;; back up one spot for the patch below
            let temp_previous_index position ? rows
            if (temp_previous_index > 0)
            [
              set a patch pxcor (item (temp_previous_index - 1) rows)
            ]
          ]
        ]
        ;; out of range
        if (b = nobody)
        [
          set b patch pxcor max-pycor
          set a patch pxcor (last rows)
          if (a = b)
          [
            set a patch pxcor (last but-last rows)
          ]
        ]
        if (a = nobody)
        [
          set a patch pxcor min-pycor
          ; set the bottom most patches concentration to same as the next north patch with data
          ask a
          [
            set concentration [concentration] of patch pxcor (first rows)
          ]
        ]
        let d_ab abs ([pycor] of b - [pycor] of a)
        let d_a abs (pycor - [pycor] of a)
        let d_b abs ([pycor] of b - pycor)
        set concentration ([concentration] of a * d_b + [concentration] of b * d_a) / d_ab
        if (concentration < 0)
        [
          set concentration 0
        ]
      ]
      
      ;; get the plume x coordinate
      let plume_center_patch max-one-of patches [concentration]
      set plume_center (list ([pxcor] of plume_center_patch) ([pycor] of plume_center_patch))
    ]
  ]
  
  ;; pedestrian exposure
  ask roads
  [
    ;; increment the exposure for each step by t * C^1.5 so exposure is like an integral of concentration^1.5 vs time
    set pedestrian_exposure pedestrian_exposure + (0.5 * concentration ^ 1.5)
    ;; 50% AEGL-2 is all we care about for noticable effects to drivers
    if ( pedestrian_exposure > AEGL_2 * 0.5 )
    [
      set pedestrians_AEGL2? true
      ;; $@GUI
      set plabel "@"
      ;; $@END
    ]
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Update Concentration
;; @Turtle
;;
;; Updates the interior concentration of the vehicle based on external concentrations
;; Updates the exposure time, keeping a history of previous exposure
;; Affects cars and garages (garages save data to pass onto new cars they spawn)
;;
;; @pre  emergency is occuring
;; @post turtle may increase or decrease concentration
;; @post turtle may increase exposure
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to update-concentration
  ;; conc(t + 1) = conc(t) + alpha * conc_outside >= 0
  set vehicle_conc max (list 0 (vehicle_conc + AIR_EXCHANGE * (concentration - vehicle_conc)))
  if-else vehicle_conc > 0
  [
    ;; time of exposure, could be smarter
    ;; increment the exposure for each step by t * C^1.5 so exposure is like an integral of concentration^1.5 vs time
    set exposure exposure + (0.5 * vehicle_conc ^ 1.5)
  ]
  [
    ;; anything?
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Check AEGL
;; @Turtle
;;
;; Check if this car has been affected by AEGL levels based on exposure history
;;
;; @pre  emergency is occuring
;; @post turtle may become disabled
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to check-aegl
  if-else (exposure > susceptibility * AEGL_3)
  [
    set incapacitated? true
    set aeglevel 3
    ;; $@GUI
    set label-color red
    set label "3"
    ;; $@END
  ]
  [
    if-else (exposure > susceptibility * AEGL_2)
    [
      set incapacitated? true
      set aeglevel 2
      ;; $@GUI
      set label-color orange
      set label "2"
      ;; $@END
    ]
    [
      if (vehicle_conc > susceptibility * AEGL_1)
      [
        set aeglevel 1
        ;; $@GUI
        set label-color yellow
        set label "1"
        ;; $@END
      ]
    ]
  ]
end

;################################################################################
;; Right Turners
;################################################################################

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Turn Right
;; @Turtle
;;
;; Complete a right turn as long as the turtle is in the intersection in the right patch
;;
;; @pre  turtle is in intersection (other requirements are checked by this procedure)
;; @post turtle will turn 90 degrees right and center itself on the patch if possible
;;  update-goal should be called after turning
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to turn-right
  ;; desired turn degree
  let turn (heading + 90) mod 360
  
  ;; ensure that the turtle is in the correct lane
  ;; behind the turtle should be non-intersection
  ifelse (not [intersection?] of patch-at-heading-and-distance heading -1 and
   [pcolor] of (patch-at-heading-and-distance (heading + 135) 1.42) = COLOR_NOT_ROAD and
    member? turn [flow_dir] of patch-here)
  [
    move-to patch-here ;; center on patch first
    set heading turn
    ;; this should be safe because update-goal is called after make-turn-decision
    set turn_goal 0 ;; no weird loops and turns, reset goal
  ]
  [
    ;; shouldn't happen, the car should be allowed to turn
    ;; $@DEBUG
    with-local-randomness
    [
      print (word "    " ticks ": " who ": I couldn't turn right, s = " speed ", a = " acceleration 
        ", lane = " ([pcolor] of (patch-at-heading-and-distance (heading + 135) 1.42) = COLOR_NOT_ROAD))
    ]
    ;; $@END
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Right Turn Lane?
;; @Turtle
;;
;; Check to see if the turtle is in the right most lane so that it can turn
;;
;; @report whether the turtle is in the right turn lane or not
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report right-turn-lane?
  report [pcolor] of patch-right-and-ahead 90 1 = COLOR_NOT_ROAD
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Right Turn Clear?
;; @Turtle
;;
;; Check to see if the turtle can make a right turn and safely exit the intersection
;; Tries to avoid blocking the intersection if the turn cannot be completed
;; This is a simple check to make sure there is a space to move to
;;
;; @report whether the traffic going out of the turn is clear
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report right-turn-clear?

  let p patch-ahead 1
  let h heading + 90
  let empty_spaces 0 ;; number of empty patches past intersection
  
  ;; number of cars in between to add to the number to look ahead
  let car_count 1 ;+ count cars-on p
  if (any? cars-on p)
  [
    report false
  ]
  
  ;; look to place immediately after turn
  ask p
  [
    set p patch-at-heading-and-distance h 1
  ]
  
  set car_count car_count + count cars-on p
  if (any? cars-on p)
  [
    report false
  ]
  
  while [empty_spaces < car_count]
  [
    ask p
    [
      set p patch-at-heading-and-distance h 1
    ]
    
    ;; check for wrong direciton, stoped, and cars waiting in long lines
    if (any? (cars-on p) with [
      speed = 0 or 
      acceleration < 0 or 
      queue > BOX_BLOCK_QUEUE_LIMIT])
    [
      report false
    ]
    if (not any? cars-on p)
    [
      set empty_spaces empty_spaces + 1
    ]
    
    ;; this means we have looked and there are not enough empty spaces
    if ([intersection?] of p)
    [
      report false
    ]
  ]
  
  ;; if we made it through that loop, traffic is clear
  report true
end

;################################################################################
;; Lane Changing
;################################################################################

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Change Lanes
;; @Turtle
;;
;; Make the turtles change lanes according to speed
;; Slow turtles will generally move right, fast turtles will tend to move left
;; if a turtle is behind a turtle going slower than it's speed limit, it will try to go around
;; Also allows turtles to change lanes in an intersection when they are stuck
;; Normally, not allowed in intersections
;;
;; @post turtle may change lanes
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to change-lanes
  if-else (is-number? passing and 
    (
      ((heading = DIR_WEST or heading = DIR_EAST) and abs (passing - xcor) > 1) or
      ((heading = DIR_NORTH or heading = DIR_SOUTH) and abs (passing - ycor) > 1)
      )
    )
  [
    ;; cars that are passing in legal lane can override the pass and continue as normal
    if (member? heading flow_dir)
    [
      set passing false
    ]
    ;; car is passing, try to get back in correct lane
    ;; skip for left_turners who are already in correct lane, but let right turners change lanes even in legal lane
    ;; @todo check for room to fit maybe?
    if (not any? cars-on patch-right-and-ahead 90 1 and
      is-number? passing and 
      member? heading [flow_dir] of (patch-right-and-ahead 90 1)
      )
    [
      ;; change lanes right
      right 90
      jump 1
      left 90
      set wait_time 0
      set passing false
      ;; $@LOGVERBOSE
      show (word "Done passing at " ticks)
      ;; $@END
      ;; $@DEBUG
      if (not member? patch-here roads)
      [
        set error_text "***ERROR Off road passing"
        show error_text
        set EXCEPTION? true
        log-error
        stop
      ]
      ;; $@END
    ]
  ]
  [
    ;; get the nearest turtle only if it is very close (note if turtle is nobody, then distance doesn't matter
    ;; @todo 1.5 used to be 1.2 but didn't work if cars weren't close enough!!! Is 1.5 enough???
    let c find-nearest-car 1.5 heading patch-here
    ;; this is normal lane changing
    ;; allow right turners doing a pass to use this if need be
    if ((turn_goal = 0 and not is-number? passing) or 
      (is-number? passing and member? heading flow_dir and wait_time > 2)
      )
    [
      if (not [intersection?] of patch-here and [pcolor] of patch-here != red)
      [
        ifelse (speed > SPEED_LIMIT and
          random-float 1 < PERCENT_LANE_CHANGE and
          change-lanes-left)
        [
          ;; fast traffic will stay in left lane
          ;; this is a "hack" to call a reporter as a normal command
          ;; return value doesn't matter
          ;; the random will only call the second command if true
          ;; if (random 100 < 10 and change-lanes-left) []
        ]
        [
          ifelse (my_speed_limit < SPEED_LIMIT and
            random-float 1 < PERCENT_LANE_CHANGE and 
            change-lanes-right)
          [
            ;; slow traffic keep right
            ;;if (random 100 < 10 and change-lanes-right) []
          ]
          [
            ;; @todo check cars a few spaces ahead to change lanes sooner and avoid slowing down
            if ((c != nobody and
              my_speed_limit > [speed] of c) or
              ;; what if we are slowed down by some other condition...like car stopped just beyond intersection
              wait_time > PASS_WAIT_TIME)
            [
              ;; try to get around turtles ahead only if there is room nearby and the move will be benefitial
              ;; do not change lanes if there will be another turtle in the way (ahead and left or ahead and right)
              let changed false
              ;; @todo these "not any?" lines are imperfect and should probably check for openings a little more precisely
              if (not any? cars-on patch-left-and-ahead 90 1 and
                not any? cars-on patch-left-and-ahead 45 1.42)
              [
                set changed change-lanes-left
              ]
              if (not changed)
              [
                if (not any? cars-on patch-right-and-ahead 90 1 and
                  not any? cars-on patch-right-and-ahead 45 1.42 and
                  change-lanes-right)
                []
              ]
            ]
            
            ;; fix for "block the box" avoidance
            if (speed = 0 and
              [pcolor] of patch-here = green and
              not intersection-clear? and
              wait_time > BOX_BLOCK_QUEUE_LIMIT
              )
            [
              ;; randomly try to change lanes left otherwise try to change right
              if-else (random-float 1 < 0.5 or not change-lanes-left)
              [
                if (change-lanes-right)
                [
                  ;; $@LOGVERBOSE
                  show (word "Can't move or will block the box, changed lanes at " ticks)
                  ;; $@END
                ]
              ]
              [
                ;; $@LOGVERBOSE
                show (word "Can't move or will block the box, changed lanes at " ticks)
                ;; $@END
              ]
            ]
          ]
        ]
      ]
    ]

    ;; help that guy that gets stuck behind right turn lane
    ;; in the intersection who can't change lanes
    ;; Or for gridlocks
    if ([intersection?] of patch-here and
      speed = 0 and
      wait_time > BOX_BLOCK_QUEUE_LIMIT)
    [
      if-else (not change-lanes-left)
      [
        if (change-lanes-right)
        [
          ;; $@LOGVERBOSE
          show (word "Stuck in intersection, changed lanes at " ticks)
          ;; $@END
        ]
      ]
      [
        ;; $@LOGVERBOSE
        show (word "Stuck in intersection, changed lanes at " ticks)
        ;; $@END
      ]
    ]
    
    ;; passing on left
    ;; wait 10 seconds and check to see that car ahead is passed out
    ;; wait time decreases linearly with sense_of_urgency
    let stress_wait_limit PASS_WAIT_TIME * (1 - sense_of_urgency)
    if (plume? and
      (wait_time > stress_wait_limit or (ahead_passed > 0 and wait_time > (stress_wait_limit / 2))) and
      c != nobody and
      not is-number? passing and
      (left-turn-lane? or turn_goal = 90) and
      [incapacitated?] of c and
      pass-on-left ;; here we call the function
      )
    [
      ;; I passed on the left
      ;; $@LOGVERBOSE
      show (word "I passed on the left at " ticks ", waited: " wait_time)
      ;; $@END
    ]
    
    ;; decrement counter for car ahead has passed, don't keep this flag on forever
    if (ahead_passed > 0)
    [
      set ahead_passed ahead_passed - 1
    ]
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Lane is Clear
;; @Turtle
;;
;; Check to see if the lane over (left or right) is clear to change lanes
;; looks at acceleration and velocity to see if car that will be behind can stop (if needed)
;; and if this car can stop for the car that will be ahead
;; basically checks other cars nearby to see if any collisions would occur
;;
;; @param spot the spot the car will go to to change lanes
;; @param opposite? true if the car will turn around so we should reverse its heading
;; @report whether or not the car can change lanes based on traffic
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report lane-is-clear? [spot opposite?]
  ;; how far back we would ever have to look assuming person is going _max speed_ (2)
  ;; s = vi * t + 1/2 a * t^2
  ;let spaces_behind 2 * ((speed - 2) / MAX_ACCELERATION) + (1 / 2) * MAX_ACCELERATION * ((speed - 2) / MAX_ACCELERATION) * ((speed - 2) / MAX_ACCELERATION)
  let time_behind ceiling ((2 - speed) / MAX_ACCELERATION)
  let spaces_behind 2 * time_behind + MAX_ACCELERATION / 2 * time_behind * time_behind + 1
  ;; how far ahaed to look
  let spaces_ahead brake-distance
  
  ;; can reverse "ahead" and "behind"
  let forward-dir heading
  let reverse-dir (heading + 180) mod 360
  if (opposite?)
  [
    set forward-dir reverse-dir
    set reverse-dir heading
  ]
  
  ;; the cars ahead and behind, over a lane
  let car_behind find-nearest-car spaces_behind reverse-dir spot
  let car_ahead find-nearest-car spaces_ahead forward-dir spot
  
  ;; will cars behind be able to stop?
  if (car_behind != nobody)
  [
    ;; calculate distance between them (but one lane over)
    let dist linear-distance car_behind ((heading + 180) mod 360)
    ;; check whether they _will_ collide
    if (will-cars-collide? car_behind self dist)
    [
      report false
    ]
  ]
  
  ;; will this car be able to slow down for cars ahead?
  if (car_ahead != nobody)
  [
    ;; fix rapid lane changes while stopped
    ;; @todo can this just be removed or moved somewhere else???
    if (speed = 0 and not opposite?)
    [
      report false
    ]
    
    ;; calculate distance between them
    let dist linear-distance car_ahead heading
    ;; check whether they _will_ collide
    if (will-cars-collide? self car_ahead dist)
    [
      report false
    ]
  ]
  
  ;; no report false, must be okay
  report true
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Will Cars Collide
;; @Observer
;;
;; Checks to see if two cars will collide based on speed, acceleration, and initial distance
;; treats car1 acceleration as -MAX_ACCELERATION to simulate the first car braking
;;
;; @param car1 the car that is behind
;; @param car2 the car that is ahead
;; @param dist the distance between the cars
;; @pre car1 is behind car2
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report will-cars-collide? [car1 car2 dist]
  ;; only check if first car is faster
  if ([speed] of car1 > [speed] of car2)
  [
    ;; assume t is time to stop car1
    ;let t ([speed] of car1) / (MAX_ACCELERATION)
    ;let dist_car1 ([speed] of car1 * t) + ((- MAX_ACCELERATION) / 2 * (t * t))
    ;let dist_car2 ([speed] of car2 * t) + ([acceleration] of car2 / 2 * (t * t))
    ;report dist < 1.1 or dist_car1 >= dist_car2 + dist
    
    ;; quadratic equation, check if t is real
    ;; (v2 - v1)^2 >= 2(a2 - a1)(s2 - s1)
    report ( ([speed] of car1 - [speed] of car2) * ([speed] of car1 - [speed] of car2) >= 
      2 * abs (AVG_ACCELERATION + MAX_ACCELERATION) * dist)
  ]
  if (dist < 1)
  [
    ;; cars would be on top of each other
    report true
  ]
  report false
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Change Lanes Right
;; @Turtle
;;
;; Attempt to change lanes to the right
;; Checks to see if the lane is clear and if it is legal to go to it
;; (correct direction, is a road)
;; Changes lanes by turning right, moving forward 1 and turning left
;; This ensures turtle does not move backward/forward, just to side
;;
;; @report true if turtle did change lanes and false if not
;; @post turtle will change lanes if possible
;; @note this must be used in a conditional and the lane change is a _side effect_
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report change-lanes-right
  ;; check if there is a road to the right
  ;; DONT assume it is the same flow dir, check it
  ;; check for no cars
  let p patch-right-and-ahead 90 1
  if ([pcolor] of p != COLOR_NOT_ROAD and
    member? heading [flow_dir] of p and
    not any? cars-on p and
    lane-is-clear? p false)
  [
    right 90
    fd 1
    left 90
    ;; car is moving
    set wait_time 0
    report true
  ]
  
  report false
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Change Lanes Left
;; @Turtle
;;
;; Attempt to change lanes to the left
;; Checks to see if the lane is clear and if it is legal to go to it
;; (correct direction, is a road)
;; Changes lanes by turning left, moving forward 1 and turning right
;; This ensures turtle does not move backward/forward, just to side
;;
;; @report true if turtle did change lanes and false if not
;; @post turtle will change lanes if possible
;; @note this must be used in a conditional and the lane change is a _side effect_
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report change-lanes-left
  ;; check if there is a road to the left, if it is the same flow dir
  ;; check for no cars
  let p patch-left-and-ahead 90 1
  if ([pcolor] of p != COLOR_NOT_ROAD and ;; color check is safe because of flow_dir check below
    member? heading [flow_dir] of p and
    not any? cars-on p and
    lane-is-clear? p false and
    ;; look to make sure cars about to turn left don't change lanes into a left turner
    ;; this is the old version but who cares, it's a random cludge...
    (turn_goal != -90 or not any? left_turners-on ((patch-set 
      (patch-left-and-ahead 45 1.42)
      (patch-left-and-ahead 90 1)
      (patch-left-and-ahead 135 1.42)))
    )
    )
  [
    left 90
    fd 1
    right 90
    ;; car is moving
    set wait_time 0
    report true
  ]
  
  report false
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Pass on Left
;; @Turtle
;;
;; Attempt to change lanes to the left into opposing traffic
;; in order to go around passed out drivers
;;
;; @report true if turtle did change lanes and false if not
;; @pre turtle has been waiting long enough to assume driver ahead is incapacitated
;; @post turtle will change lanes if possible
;; @note this must be used in a conditional and the lane change is a _side effect_
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report pass-on-left
  ;; cars can only pass into intersection when the light is green and the box block check is done
  ;; make sure there is an opposite lane to go to
  ;; this MUST be a road AND must be opposing traffic (NOT cross traffic)
  if (
    ([pcolor] of patch-at-heading-and-distance (heading - 90) 1 = COLOR_ROAD or
      [pcolor] of patch-at-heading-and-distance (heading - 90) 1 = COLOR_INTERSECTION or
      [pcolor] of patch-at-heading-and-distance (heading - 90) 1 = green) and ;; off chance they want to pass just past intersection
    (member? ((heading + 180) mod 360) ([flow_dir] of patch-at-heading-and-distance (heading - 90) 1) or ;; no cross traffic bug fix
      	  member? heading ([flow_dir] of patch-at-heading-and-distance (heading - 90) 1)
      )
    )
  [
    ;; start at current patch just because of those weird patch boundaries
    let i 0
    ;; begin with nearest car
    let car_ahead find-nearest-car safe-distance heading patch-here
    let okay? false
    
    ;; make sure all cars ahead are incapacitated and then find an empty space to move to
    while [not okay? and (car_ahead = nobody or [incapacitated?] of car_ahead) and i <= PASS_DIST_MAX]
    [
      set i i + 1
      ;; find an empty space
      if-else (not any? cars-on patch-ahead i)
      [
        set okay? true
      ]
      [
        set car_ahead one-of cars-on patch-ahead i
      ]
    ]
    
    if (okay?)
    [
      ;; check oncoming traffic for a 2n+3 break
      ;; the plus three because it takes 2 spaces to get up to speed and 1 space cushion
      let space patch-left-and-ahead 90 1
      let dir heading
      let cnt i ;; cars to pass count
      let lim i * 2.5 + 5.5
      set i 1
      while [i <= lim]
      [
        let curr (cars-on space)
        if (any? curr)
        [
          ;; could be disabled car far enough away, as long as there is empty space before it
          if-else (i > cnt  and (any? curr with [incapacitated?])
            )
          [
            ;; good to go, kill the loop
            set i lim
          ]
          [
            if-else ([heading] of curr = ((heading + 180) mod 360))
            [
            ;; one last speed check for collisions
            ;; car must move n + 1 patches (where n is the number of cars it's passing)
            ;; this will take n + 3 ticks
            ;; assume oncoming speed capped at 1.5 (rough estimate, ignore speeders for now)
            ;; must look (n + 1) + (n + 3) * 1.5(speed) patches ahead for cars
            ;; now if a car is found, check it's max speed (assume if speed < max, it will accelerate to max quickly)
            ;; if speed < (i - n - 1)/(n + 3) then it is going slow enough we can make it
            if-else ([my_speed_limit] of one-of curr < (i - cnt - i) / (cnt + 3))
            [
              ;; should be okay, won't collide (assuming no opposing cars pass that one and overtake it)
              set i lim
            ]
            [
              report false
            ]
          ]
            [
              ;; car we see is in same direction as this car
              ;; don't go if it is incapacitated
              if ([heading] of curr = heading and [incapacitated?] of curr)
              [
                report false 
              ]
              ;; cross traffic is probably dangerous, let's wait for it to stop
              if ([heading] of curr != heading)
              [
                report false
              ]
            ]
          ]
        ]
        set i i + 1
        ;; move up to next space
        ask space
        [
          set space patch-at-heading-and-distance dir 1
        ]
      ]
      
      ;; everything is okay, good to go
      ;;notify driver behind (because driver behind will not run until after this car has changed lanes)
      let behind find-nearest-car 2 ((heading + 180) mod 360) patch-here
      if (behind != nobody)
      [
        ask behind
        [
          ;; $@LOGVERBOSE
          show (word "I see the car ahead making a pass at " ticks)
          ;; $@END
          set ahead_passed PASS_WAIT_TIME * 2
        ]
      ]
      ;; change lanes
      left 90
      fd 1
      right 90
      ;; $@DEBUG
      if (not member? patch-here roads)
      [
        set error_text "***ERROR Off road passing called in pass-on-left"
        show error_text
        set EXCEPTION? true
        log-error
        stop
      ]
      ;; $@END
      set passing ifelse-value (heading = DIR_WEST or heading = DIR_EAST) [xcor] [ycor]
      report true
    ]
  ]
  ;; failure condition
  report false
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Maybe Make U-Turn
;; @Turtle
;;
;; Make a U-Turn if it is appropriate
;;
;; @post turtle will make u-turn if it should
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to maybe-make-u-turn
  if (u_turns? and speed = 0
    and breed != left_turners
    and sense_of_urgency > law_compliance_threshold
    and sense_of_urgency < tunnel_vision_threshold
    and wait_time > U_TURN_WAIT_TIME
    and make-u-turn)
  [
    ;; $@LOGVERBOSE
    show (word "&&& Making a U-Turn at " ticks)
    ;; $@END
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Make U-Turn
;; @Turtle
;;
;; Make a U-Turn by changing lanes to the left into the opposite direction and changing direction
;;
;; @report true if turtle did make u-turn and false if not
;; @post turtle will make u-turn if possible
;; @note this must be used in a conditional and the lane change is a _side effect_
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report make-u-turn
  ;; check if there is a road to the left, if it is the opposite flow dir
  ;; check for no cars
  let p patch-left-and-ahead 90 1
  if ([pcolor] of p != COLOR_NOT_ROAD and ;; color check is safe because of flow_dir check below
    member? ((heading + 180) mod 360) [flow_dir] of p and
    not any? cars-on p and
    lane-is-clear? p true
    )
  [
    left 90
    fd 1
    left 90
    ;; car is moving
    set wait_time 0
    
    ;; need to reevaluate goal (now heading to a different intersection)
    update-curr-goal corrected-heading
    ;; now make a new decision
    update-goal true
    
    report true
  ]
  
  report false
end

;################################################################################
;; Left Turners
;################################################################################

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Left Turn Lane?
;; @Turtle
;;
;; Check to see if the turtle is in the left most lane so it can turn left
;; This means the lane to the left is the opposite direction or a non road (one-way roads)
;;
;; @report whether the lane is the farthest left it can be in
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report left-turn-lane?
  ;; REMEMBER, flow_dir is a LIST
  report [pcolor] of patch-at-heading-and-distance (heading - 90) 1 = COLOR_NOT_ROAD or 
    not member? heading ([flow_dir] of patch-at-heading-and-distance (heading - 90) 1)
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Prepare Left Turn
;; @Turtle : (car)
;;
;; If the car wants to turn left, it will try to get into the turn lane
;; This will turn the car into a left_turner
;; This happens if there is a left turn line ahead or if the car ahead is stopped
;; This way the turner will get in the "lane" to pass stopped traffic
;; Avoid long lines of turners
;;
;; @pre  turtle wants to turn left and is in the left lane
;; @pre  this will not be called on a car that is "passing"
;; @post turtle may turn into a left_turner
;; @post turtle may change goal to go straight if the line is too long and try to change lanes right
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to prepare-left-turn
  let car_ahead find-nearest-car brake-distance heading patch-here
  let turner_ahead find-nearest-left-turner brake-distance heading patch-here
  
  ;; if there are stopped cars ahead, the light is ahead, or there are other turners in line ahead; prepare
  ;; do not change into a left turner in the middle of intersections
  if (left-turn-lane? and
    (
      (car_ahead != nobody and
        [speed] of car_ahead = 0 and pcolor = white) or
      ([pcolor] of patch-ahead 1 != white) or
      ([pcolor] of patch-ahead 2 != white) or
      turner_ahead != nobody
    )
    )
    
  [
    ifelse (turner_ahead != nobody and [queue] of turner_ahead >= GRID_Y_SIZE / 2)
    [
      ;; the queue is too long, don't get in line, go to next intersection
      set turn_goal 0
      ;; get in right lane if possible
      if (change-lanes-right) []
    ]
    [
      ;; we are about to be stopped, lets bypass it
      set breed left_turners
      set ready? false
      set queue 0
      ;; is anyone ahead?
      if (turner_ahead != nobody)
      [
        set queue ([queue] of turner_ahead + 1)
      ]
    ]
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Continue Left Turn
;; @Turtle : (left_turner)
;;
;; This procedure makes left_turners move forward up to the light
;; They will proceed into the intersection on a green or yellow light, providing that
;; the traffic leaving the turn is clear and they will not "block the box"
;; They will complete their turn when in the intersection and the light turns yellow
;; or traffic is clear
;; left_turn speeds decrease as long as they wait in line
;;
;; @pre breed = left_turners
;; @post turtle will try to move forward until it can turn left at which point it becomes a normal car
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to continue-left-turn
  ;; sometimes the queue is broken, try to repair it
  let turner_ahead find-nearest-left-turner brake-distance heading patch-here
  ifelse (turner_ahead != nobody)
  [
    set queue ([queue] of turner_ahead + 1)
  ]
  [
    set queue 0
  ]
  
  ;; desired direction after the turn
  let turn (heading - 90) mod 360
  
  ifelse (not ready?)
  [
    ;; let's keep normal rules until they reach the intersection,
    ;; so if they are on the road and not about to get on a light patch, they can still drive fast
    ifelse (pcolor = white and (color-of-nearest-light safe-distance) = black)
    [
      ;; check to see if the turtle recently recieved evac orders and changed turn_goal
      ;; they can still back out of left turn if traffic is clear
      ifelse (turn_goal != -90 and not any? cars-on patch-here)
      [
        set breed cars
        ;; might be a good idea to change lanes to "get out of the way"
        if (change-lanes-right) []
      ]
      [
        if-else (turner_ahead != nobody)
        [
          let data (speed-car-ahead turner_ahead)
          if-else (last data)
          [
            brake MAX_ACCELERATION
          ]
          [
            ;if-else (item 1 data)
            ;[
            ;  brake AVG_ACCELERATION
            ;]
            ;[
              accelerate AVG_ACCELERATION
            ;]
          ]
        ]
        [
          accelerate AVG_ACCELERATION
        ]
        apply-acceleration
        jump speed
      ]
    ]
    [
      ;; why not center on the patch
      move-to patch-here
      
      ;; still on the road, not intersection
      ;; possibly dangerous, let cars keep turing on yellow lights, hope it works
      ifelse ( (pcolor = white or pcolor = green or pcolor = yellow) and
        not any? left_turners-on patch-ahead 1)
      [
        ;; need a mod to NOT go into intersection when it is backed up
        let h heading
        let x item 0 curr_goal
        let y item 1 curr_goal
        
        ;; only do this on green if there are no opposite turners in the intersection
        ;; also check that the turners can complete turn
        ifelse (pcolor = white or
          ((pcolor = yellow or pcolor = green) and
            (
              ;; check for opposite turners in intersection
              not any? left_turners with
              [
                [column] of patch-here = x and [row] of patch-here = y and
                (heading = (h + 90) mod 360 or heading = (h - 90) mod 360)
              ] and
              not any? (cars-on patch-ahead 1) with
              [
                heading != h
              ] and
              can-complete-left-turn?
            )
          )
        )
        [
          ;; keep moving forward until we get on the light
          set speed min (list 1 my_speed_limit)
          move-to patch-ahead 1
          set wait_time 0
        ]
        [
          set speed 0
        ]
      ]
      [
        let h heading
        ifelse ([intersection?] of patch-here and
          (not any? left_turners-on patch-ahead 1) and
          (not any? (cars-on patch-ahead 1) with
          [
            heading != h
          ]) and
          not (member? turn [flow_dir] of patch-here))
        [
          ;; keep moving forward until we are in the lane to turn into
          set speed min (list 1 my_speed_limit)
          move-to patch-ahead 1
          set wait_time 0
        ]
        [
          set speed 0
        ]
        if (member? turn [flow_dir] of patch-here)
        [
          set ready? true
        ]
      ]
    ]
  ]
  [
    ;; ready? = true
    ;; now we see if traffic is clear
    ;; as long as it is clear, we can turn, move turtle forward one, and treat it as a normal car again
    ;; remember to ignore left_turners in opposing traffic, need to just go through them for simplicity
    
    ;; if the light is red, get out of there as soon as there are not any cars in the way
    ifelse (
      (not any? cars-on (patch-at-heading-and-distance turn 1)) and
      (
        ;; checks for red lights
        (
          ((heading = DIR_WEST or heading = DIR_EAST) and
            (([light_h] of ([controller] of patch-here) = red))) or
          ((heading = DIR_NORTH or heading = DIR_SOUTH) and
            (([light_v] of ([controller] of patch-here) = red)))
        )
        or
          left-traffic-clear?
      )
    )
    [
      move-to patch-here ;; fix bug where some left_turners get on non integer path, wierd offset
      set heading turn
      set breed cars
      ;; give cars speed = 1
      reset-acceleration
      set speed min (list 1 my_speed_limit)
      jump speed
    ]
    [
      set speed 0
    ]
  ]
  
  ;; what if the left turn lane is blocked by AEGL-2 cars?
  if-else ((wait_time > PASS_WAIT_TIME or (ahead_passed > 0 and wait_time > PASS_WAIT_TIME / 2)) and
    turner_ahead != nobody and
    [incapacitated?] of turner_ahead)
  [
    ;; trick is to make the turner behave like a car so passing can kick in if possible and it will turn back into turner later
    if (not any? cars-on patch-here)
    [
      ;; $@LOGVERBOSE
      show (word "Left turner making pass at " ticks)
      ;; $@END
      set breed cars
      ;; let the passing mechanism force the car to move up one space before getting back into a line of stuck turners
      set passing ifelse-value (heading = DIR_WEST or heading = DIR_EAST) [xcor] [ycor]
      ;; just an extra option
      if (change-lanes-right) []
    ]
  ]
  [
    ;; what about turners waiting the entire time (almost)?  They should probably stop trying to turn...
    if (wait_time > MAX_WAIT_TIME and not any? cars-on patch-here)
    [
      ;; $@LOGVERBOSE
      show (word "Left turner waited too long at " ticks)
      ;; $@END
      ;; simplest fix
      set turn_goal 0
      if (can_change?)
      [
        ;; well they wanted to turn, maybe they can turn right instead
        set turn_goal 90
        ;; can't go left though
        set can_change? false
        ;; make sure it's legal (obviously no check for left turns)
        fix-one-way heading
      ]
      set breed cars
    ]
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Left Traffic Clear?
;; @Turtle : (left_turner)
;;
;; Check to see if a left turner ready to turn can complete turn
;; by looking at oncoming opposing traffic
;; Slow algorithm looks at a "stair-step" pattern of patches to predict
;; which cars will be in the way when the turtle turns
;;
;; @pre  left_turner is "ready?" to turn
;; @report true if the traffic is clear and it is safe to go
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report left-traffic-clear?
  let clear? true
  let angle (heading - 90)
  ;; lanes car must pass in completing turn
  let lanes LEFT_CHECK_DIST
  ;; patch temporary to walk through
  let next patch-left-and-ahead 90 1
  let temp nobody
  
  while [clear? and [intersection?] of next]
  [
    ask next
    [
      let i 0
      while [i <= lanes]
      [
        ;; are there any cars there?
        ;; if they are at a green light and stopped, 
        if (any? (cars-on (patch-at-heading-and-distance (angle + 90) i)) with
          [(pcolor = green and (acceleration != (- MAX_ACCELERATION) or not intersection-clear?)) or
            (speed + acceleration > AVG_ACCELERATION and brake-distance >= i) or
            ;; fix to check for cars stuck in intersection (although they should avoid it)
            ([intersection?] of patch-here and speed < MAX_ACCELERATION)
          ])
        [
          ;; the car approaching can't brake in time
          set clear? false
          stop ;; breaks ask block
        ]
        set i i + 1
      ]
      set lanes lanes + 2
      ;; look to next patch
      set temp patch-at-heading-and-distance angle 1
    ]
    set next temp
  ]
  report clear?
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Can Complete Left Turn?
;; @Turtle : (left_turner)
;;
;; Check to see if a turner can enter the intersection and exit safely
;; Looks at all turners ahead and counts spaces they can move to after turning
;; Avoids "blocking the box"
;;
;; @pre  breed = left_turners
;; @pre  pcolor = green or yellow (must be on a traffic light space just before intersection)
;; @report true if the turtle can go through safely
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report can-complete-left-turn?
  ;; find the patch outside of intersection
  let p patch-here
  let i 0
  ;; play it safe, look for a lot of space so they don't block intersection
  let car_count LEFT_BOX_BLOCK_BUFFER
  let empty_spaces 0 ;; spaces available to go to
  ;; ignore only a couple of disabled cars
  let disabled_count 0
  let turn (heading - 90) mod 360
  
  ;; get to point where car can turn
  while [not (member? turn [flow_dir] of p)]
  [
    set i i + 1
    set p patch-ahead i
    ;; count number of cars ahead
    set car_count car_count + count left_turners-on p
  ]
  
  ;; now we turn to the left and walk out of the intersection
  while [[intersection?] of p]
  [
    ask p [set p patch-at-heading-and-distance turn 1]
    ;; count all cars travelling in direction this car will travel
    set car_count car_count + count (cars-on p) with [heading = turn]
  ]
  
  ;; now p is just outside of intersection
  if (any? (cars-on p) with [queue > BOX_BLOCK_QUEUE_LIMIT or speed = 0])
  [
    ;; don't go if the spot just outside the intersection is full
    report false
  ]
  
  ;; empty space count
  if (not any? cars-on p)
  [
    set empty_spaces empty_spaces + 1
  ]
  
  ;; counter to limit search
  let counter 1
  
  ;; look ahead and count how many cars approximately will be filling in to new spot
  ;; if there is room for this car and all cars ahead of it to complete turn, they may proceed
  while [empty_spaces < car_count]
  [
    ask p [set p patch-at-heading-and-distance turn 1]
    ifelse (any? (cars-on p) with [queue > BOX_BLOCK_QUEUE_LIMIT or speed = 0])
    [
      if-else (empty_spaces > 1 and any? (cars-on p) with [queue > BOX_BLOCK_QUEUE_LIMIT or speed = 0 and incapacitated?])
      [
        ;; eh, we can probably safely slide by?
        ;; @todo needs more testing could be unsafe
        set empty_spaces empty_spaces - 1
        set disabled_count disabled_count + 1
      ]
      [
        report false
      ]
    ]
    [
      ;; empty space count
      if (not any? cars-on p)
      [
        set empty_spaces empty_spaces + 1
      ]
    ]
    
    ;; terminate when we reach the next intersection
    if ([intersection?] of p)
    [
      report false
    ]
    
    ;; limit search to half of an average horizontal block
    set counter counter + 1
    if (counter > GRID_X_SIZE / 2)
    [
      report false
    ]
    
    ;; too many disabled is probably a bad idea
    if (disabled_count > PASS_DIST_MAX)
    [
      report false
    ]
  ]
  
  ;; did not report false so safe to say true
  report true
end

;################################################################################
;; Goals and Destinations
;################################################################################

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Make Turn Decision
;; @Turtle
;;
;; Calls various procedures based on the turtle's turn goal
;; will make left turners get in left lane and keep trying to prepare-left turn until they are left_turners breed
;; moves right turners to right lane and slows them down as they approach intersection
;; causes right turners to complete their turn when they are on the intersection
;;
;; @pre  update-goal _must_ _not_ be called _first_
;;   doing so will break right hand turns because they will update their goal before they have completed the turn
;; @post turtle _may_ change lanes, prepare a left turn, or make a right turn.
;;       also, may change turn goal, such as if going straight and waiting for blocked box -> turn right
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to make-turn-decision
  if (not is-number? passing)
  [
    ifelse ([not intersection?] of patch-here)
    [
      ;; trying to turn left?
      ifelse (turn_goal = -90)
      [
        ifelse (left-turn-lane?)
        [
          ;; correct lane
          prepare-left-turn
        ]
        [
          ;; get into correct lane
          if (change-lanes-left) []
        ]
      ]
      [
        ifelse (turn_goal = 90)
        [
          ifelse (right-turn-lane?)
          [
            ;; correct lane
            
            ;; allow right on red here
            if (pcolor = yellow or pcolor = red)
            [
              right-on-red
            ]
          ]
          [
            ;; right turners didn't make the right turn lane
            ifelse (pcolor = green)
            [
              set turn_goal 0
            ]
            [
              ;; get into correct lane
              if (change-lanes-right) []
            ]
          ]
        ]
        [
          ;; sometimes the box is blocked for a LONG TIME, in that case, let's try to just turn right
          ;; Make sure this isn't creating another problem by checking that right turn traffic is clear
          
          if (block_time > HIGH_WAIT_TIME_TOLERANCE and
            turn_goal = 0 and
            can_change? and
            ;; only do this if we save time by turning right on red or the light is green
            (
              (
                (pcolor = yellow or pcolor = red) and
                can-turn-right-on-red?
              ) or
              pcolor = green
            ) and
            not intersection-clear? and
            right-turn-lane? and
            right-turn-clear?
            )
          [
            ;; $@LOGVERBOSE
            show (word "Car waited too long at " ticks ", _________________________________________________________________TURNING RIGHT")
            ;; $@END
            set turn_goal 90
            ;; may be illegal
            fix-one-way heading
          ]
        ]
        
      ]
    ]
    [
      ;; on an intersection patch
      ;; make sure it is the correct intersection by comparing to curr_goal
      ;; (so we don't turn immediately in intersection upon choosing goal)
      if (turn_goal = 90 and 
        (([row] of patch-here = item 1 curr_goal) and
          ([column] of patch-here = item 0 curr_goal)) )
      [
        turn-right
      ]
    ]
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Update Goal
;; @Turtle
;;
;; Update the turtle's current goal and turning goal in an effort to reach the ultimate goal
;; Only updates goal when the turtle has reached the next intersection, at which point it
;; decides how to turn at the following intersection
;; Decisions broken into sub procedures
;; This will find new goals when the turtle has reached its goal
;;
;; @param initial? when true, this will update the goal even when the turtle is not on the next intersection
;;  this is used for first runs and when a turtle changes goals unexpectedly
;;
;; @post turtle may update its goals (curr_goal, local_goal, turn_goal)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to update-goal [initial?]
  ifelse (not evac? and 
    (empty? local_goal or 
      (not empty? local_goal and
        ([row] of patch-here = item 1 local_goal) and ([column] of patch-here = item 0 local_goal))) and 
    (not leave? or (leave? and leave-goal-done?)) )
  [
    ;; Goal has been completed
    ;;set label "GOAL"
    ;;set label-color black
    
    if ((not empty? local_goal and
      ([row] of patch-here = item 1 local_goal) and ([column] of patch-here = item 0 local_goal)))
    [
      ;; need to first remove the old goal if it has just been completed
      set local_goal remove-item 0 local_goal
      set local_goal remove-item 0 local_goal
    ]
    
    ;; will find a new random goal, which may be to evacuate during an emergency (PERCENT_HEAR_AFTER)
    find-new-goal
    find-first-goal
    update-goal true
  ]
  [
    ;; adjust heading if turtle is completing a turn and is looking for goal at NEXT intersection
    ;; don't do this if the left_turner is heading to current intersection still (initial? = true)
    let new_dir corrected-heading

    ifelse (initial? or
      ([intersection?] of patch-here and
        ([row] of patch-here = item 1 curr_goal) and ([column] of patch-here = item 0 curr_goal))
      )
    [
      ;; when run initially, don't change the goal, then do this otherwise
      if (not initial?)
      [
        update-curr-goal new_dir
      ]
      
      ;; fix for intermediate goals while evacuating
      if (evac? and not empty? local_goal and not initial? and
        ([row] of patch-here = item 1 local_goal) and ([column] of patch-here = item 0 local_goal))
      [
        set local_goal remove-item 0 local_goal
        set local_goal remove-item 0 local_goal
      ]
      
      ;; different types of goals
      ifelse (evac? and empty? local_goal)
      [
        ;; evacuation routine
        evacuate new_dir
        fix-one-way new_dir
        fix-no-left new_dir
      ]
      [
        ifelse (leave? and empty? local_goal)
        [
          ;; non-local
          leave-goal new_dir
          fix-one-way new_dir
        ]
        [
          ;; local
          local-goal new_dir
          fix-one-way new_dir
          fix-no-left new_dir
        ]
      ]
    ]
    [
      if (evac? and empty? local_goal and block_time > BLOCK_MAX_WAIT)
      [
        ;; here if the car is waiting a long time to evacuate, have it head to the evacuation road
        evacuation-route new_dir
        fix-one-way new_dir
      ]
      
      if (not evac? and alarm? and (signs? or radios?))
      [
        maybe-evacuate
      ]
    ]
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Get Corrected Heading
;; @Turtle
;;
;; Return the heading of this turtle to its curr goal (correct if it is a left turner)
;; We need this because we can't rely on "initial?" in update-goal (that was a BUG)
;;
;; @report Heading that can be used in other decision making
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report corrected-heading
  if (breed = left_turners and [intersection?] of patch-here)
  [
    report ((heading - 90) mod 360)
  ]
  report heading
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Update Curr Goal
;; @Turtle
;;
;; Update the curr_goal with the next intersection that the turtle will go to
;; based on how it just turned
;;
;; @param new_dir the direction the turtle is going (or will be going if turning left)
;; used to calculate the next goal
;;
;; @pre  turtle is in the intersection of its curr_goal so it must be updated,
;; do _not_ call this for initial? = true
;; @post turtle will change its curr_goal based on the direction it is going
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to update-curr-goal [new_dir]
  ;; $@DEBUG
  ;; keep track of travel history
  set $intersection_hist fput curr_goal $intersection_hist
  ;; $@END
  
  ;; assume heading+turn is the new heading of the turtle
  ;; because if they are a left_turner, they will not be facing true direction
  ;; MUST ASSUME that right turners will change direction BEFORE this procedure or they will get "lost"
  if (new_dir = DIR_WEST) [set curr_goal replace-item 0 curr_goal (((item 0 curr_goal) - 1) mod NUM_ROADS_X)]
  if (new_dir = DIR_EAST) [set curr_goal replace-item 0 curr_goal (((item 0 curr_goal) + 1) mod NUM_ROADS_X)]
  if (new_dir = DIR_NORTH) [set curr_goal replace-item 1 curr_goal (((item 1 curr_goal) + 1) mod NUM_ROADS_Y)]
  if (new_dir = DIR_SOUTH) [set curr_goal replace-item 1 curr_goal (((item 1 curr_goal) - 1) mod NUM_ROADS_Y)]
  
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Evacuate
;; @Turtle
;;
;; Make decisions to reach the evacuation goal
;; Turtle will try to head east (or west depending on value of evac_goal)
;;
;; @param new_dir the direction the turtle is going (or will be going if turning left)
;;
;; @pre  turtle is evac? = true and empty? local_goal
;; @post turtle will change its turn_goal based on the direction it is going
;; @post turtle will not turn left on evacuation road
;; @post call fix-one-way and fix-no-left
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to evacuate [new_dir]
  ;; simple method: just go east
  ;; @todo should be smarter
  
  ;; keep track of position
  let x first curr_goal
  let y item 1 curr_goal
  
  ;; catch-all case
  set turn_goal 0
  
  ;; case to change evacuation direction
  let may_change_goal? is-aware-and-know?
  
  ;; first if going away from goal
  ifelse (new_dir = ((evac_goal + 180) mod 360))
  [
    ifelse (may_change_goal? and random-float 1 < PERCENT_EVAC_CHANGE and
      (
        (new_dir = DIR_EAST and xcor > (first plume_center) + (PLUME_POS_TOLERANCE - PLUME_POS_TOLERANCE * knowledge)) or
        (new_dir = DIR_WEST and xcor < (first plume_center) - (PLUME_POS_TOLERANCE - PLUME_POS_TOLERANCE * knowledge))
      ))
    [
      set evac_goal new_dir
      ;; $@LOGVERBOSE
      show (word "Changing evacuation direction to CURRENT direction " ticks)
      ;; $@END
    ]
    [
      ;; need to turn completely around, let's favor turning right because it is faster
      ifelse (random-float 1 < PERCENT_EVAC_RIGHT)
      [
        ;; x% chance turning right
        set turn_goal 90
        if (not member? patch-here evac_roads)
        [
          set can_change? true
        ]
      ]
      [
        ;; will be fixed on evac roads with fix-no-left
        set turn_goal -90
        set can_change? true
      ]
    ]
  ]
  [
    ;; could change directions
    let did_change_goal? false
    if (may_change_goal? and random-float 1 < PERCENT_EVAC_CHANGE and
      (
        (evac_goal = DIR_WEST and xcor > (first plume_center) + (PLUME_POS_TOLERANCE - PLUME_POS_TOLERANCE * knowledge)) or
        (evac_goal = DIR_EAST and xcor < (first plume_center) - (PLUME_POS_TOLERANCE - PLUME_POS_TOLERANCE * knowledge))
        ))
    [
      set evac_goal ((evac_goal + 180) mod 360)
      set did_change_goal? true
    ]
    
    ifelse (new_dir = evac_goal)
    [
      ;; changed goal rather than driving straight into plume, now recursively call this again
      ifelse (did_change_goal?)
      [
        ;; $@LOGVERBOSE
        show (word "Changing evacuation direction to OPPOSITE direction " ticks)
        ;; $@END
        ;; try this again recursively
        evacuate new_dir
      ]
      [
      
        set turn_goal 0
        
        ;; try to go to evacuation roads more often
        ;; this requires knowledge and lack of panic
        if (not member? y evac_list and
          (
            knowledge > EVAC_ROUTE_KNOW_MIN or knowledge + awareness > MAX_KNOW or not knowledge?
            ) and
          sense_of_urgency < tunnel_vision_threshold)
        [
          evacuation-route new_dir
        ]
      ]
    ]
    [
      ;; $@LOGVERBOSE
      if (did_change_goal?)
      [
        show (word "Changing evacuation direction (was driving PERPENDICULAR) " ticks)
      ]
      ;; $@END
      ifelse (new_dir = DIR_NORTH)
      [
        ;; heading north, so turn always if knowledge low or sense of urgency high
        ;; else turn if at edge of map (programming simplicity for me) or at evac route
        if ((knowledge? and knowledge < EVAC_ROUTE_KNOW_MIN) or
          sense_of_urgency > tunnel_vision_threshold or
          y = NUM_ROADS_Y - 1 or
          member? y evac_list)
        [
          set turn_goal ifelse-value (evac_goal = DIR_EAST) [90] [-90]
        ]
      ]
      [
        ;;ifelse (new_dir = DIR_SOUTH)
        
        ;; heading south, so turn always if knowledge low or sense of urgency high
        ;; else turn if at edge of map (programming simplicity for me) or at evac route
        if ((knowledge? and knowledge < EVAC_ROUTE_KNOW_MIN) or
          sense_of_urgency > tunnel_vision_threshold or
          y = 0 or
          member? y evac_list)
        [
          set turn_goal ifelse-value (evac_goal = DIR_EAST) [-90] [90]
        ]
      ]
    ]
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Evacuate Route
;; @Turtle
;;
;; Attempt to go to the evacuation route
;;
;; @param new_dir the direction the turtle is going (or will be going if turning left)
;;
;; @pre  turtle is evac? = true and empty? local_goal
;; @pre  We have already checked that knowledge is sufficient and sense of urgency is low enough to make this decision
;; @post turtle will change its turn_goal to reach the evacuation route if possible
;; @post turtle will not turn left on evacuation road
;; @post turtle will have new local_goal if it is going to the evacuation road
;; @post must call fix-one-way
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to evacuation-route [new_dir]
  ;; keep track of position
  let x first curr_goal
  let y item 1 curr_goal
  
  ;; nearest evac road
  let nearest 0
  ;; nearest evac road in opposite direction
  let next_nearest 0
  let i 1
  
  ;; find the nearest evac_road
  while [i < length (evac_list)]
  [
    if (abs (y - item i evac_list) < abs (y - item nearest evac_list))
    [
      set nearest i
    ]
    set i (i + 1)
  ]
  
  ;; now nearest is the nearest evac_road
  ;; don't do anything if we are on an evac road
  if (y != item nearest evac_list)
  [
    ;; compare the distance to go to the evac road and the distance to drive straight
    let dist_to_edge ifelse-value (evac_goal = DIR_EAST)
                       [max-pxcor - pxcor]
                       [pxcor - min-pxcor]
    ;; add or subtract 1 depending on direction
    let dir_factor ifelse-value (evac_goal = DIR_EAST) [1] [-1]
    
    if (random-float 1 < PERCENT_EVAC_TRY_ROUTE and
      abs (y - item nearest evac_list) * GRID_Y_SIZE < dist_to_edge * EVAC_ROAD_DIST_MOD)
    [
      ;; determine which direction to turn
      ifelse (y < item nearest evac_list)
      [
        set turn_goal -90
        ;; can we also turn right if we have to?
        set i 0
        while [i < length (evac_list)]
        [
          ;; check to see if it makes sense to turn onto another evac_road
          ;; and if we would turn the opposite direction in that case
          if (i != nearest and
            (abs (y - item i evac_list) * GRID_Y_SIZE < dist_to_edge * EVAC_ROAD_DIST_MOD) and
            y > item i evac_list
            )
          [
            set can_change? true
            set next_nearest i
          ]
          set i (i + 1)
        ]
      ]
      [
        set turn_goal 90
        ;; can we also turn left if we have to?
        set i 0
        while [i < length (evac_list)]
        [
          ;; check to see if it makes sense to turn onto another evac_road
          ;; and if we would turn the opposite direction in that case
          if (i != nearest and
            (abs (y - item i evac_list) * GRID_Y_SIZE < dist_to_edge * EVAC_ROAD_DIST_MOD) and
            y < item i evac_list
            )
          [
            set can_change? true
            set next_nearest i
          ]
          set i (i + 1)
        ]
      ]
      
      ;; don't head to plume if knowing
      if (is-aware-and-know? and is-heading-to-plume-y? (new_dir + turn_goal))
      [
        ifelse (can_change?)
        [
          set turn_goal turn_goal * -1
          set can_change? false
        ]
        [
          set turn_goal 0
        ]
        ;; $@LOGVERBOSE
        show (word "avoided driving into plume to evacuate " ticks)
        ;; $@END
      ]
      
      ;; we are trying to get to the evac road, see if the next turn is possible
      let temp_goal turn_goal
      fix-one-way new_dir ;; changes turn_goal possibly
      ;; if the turn to the evac road is allowed, set an intermediate goal to it
      ifelse (temp_goal = turn_goal)
      [
        ;; must be x +/- 1 so cars turn when they reach evac_road
        set local_goal list (x + dir_factor) (item nearest evac_list)
        ;; do not update goal yet, wait so that car naturally updates goal at next intersection after turning
        ;; $@LOGVERBOSE
        show (word "Heading to nearest evacuation road at " ticks)
        ;; $@END
      ]
      [
        ;; turn_goal changed, but if it did not change to 0, keep going
        if (turn_goal != 0)
        [
          ;; one-way road forced us to turn other way onto other evac road, next_nearest
          
          ;; don't head to plume if knowing
          ifelse (not is-aware-and-know? or not is-heading-to-plume-y? (new_dir + turn_goal))
          [
            ;; must be x +/- 1 so cars turn when they reach evac_road
            set local_goal list (x + dir_factor) (item next_nearest evac_list)
            ;; do not update goal yet, wait so that car naturally updates goal at next intersection after turning
            ;; $@LOGVERBOSE
            show (word "Heading to other evacuation road at " ticks)
            ;; $@END
          ]
          [
            ;; $@LOGVERBOSE
            show (word "avoided driving into plume to evacuate " ticks)
            ;; $@END
          ]
        ]
      ]
    ]
  ]
end


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Is Heading to Plume
;; @Turtle
;;
;; Check to see if the turtle is heading toward the plume (north/south)
;; ignore this if turtle is >PLUME_SAFE_DISTANCE away
;; This returns false when there is no plume
;;
;; @param new_dir the direction the turtle will go after applying the turn_goal
;;
;; @report true if they are heading to the plume
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report is-heading-to-plume-y? [new_dir]
  ;; exit if there is no plume
  if (not plume?)
  [
    report false
  ]
  set new_dir new_dir mod 360
  report ((new_dir = DIR_SOUTH and ycor > item 1 plume_center + (PLUME_POS_TOLERANCE - PLUME_POS_TOLERANCE * knowledge)) or
          (new_dir = DIR_NORTH and ycor < item 1 plume_center - (PLUME_POS_TOLERANCE - PLUME_POS_TOLERANCE * knowledge))) and
          (abs (ycor - item 1 plume_center) <= PLUME_SAFE_DISTANCE)
  
end
  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Is Aware
;; @Turtle
;;
;; Check to see if the turtle is sufficiently aware to evacuate (with x%)
;;
;; @report true if they are aware
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report is-aware?
  report (change_evac_goal_allowed? and
    (awareness) > AWARENESS_THRESHOLD_LOW)
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Is Aware and Knowledgable
;; @Turtle
;;
;; Check to see if the turtle is sufficiently aware and knowledgable
;; to make a smart decision, BUT not having tunnel vision due to high sense of urgency, low performance
;;
;; @report true if they are aware and knowledgable, not panicked
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report is-aware-and-know?
  report (is-aware? and
    awareness + knowledge > MAX_KNOW and
    sense_of_urgency < tunnel_vision_threshold)
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Evacuation Direction
;; @Turtle
;;
;; Figure out which direction to evacuate based on current location (assume instructions say to evacuate to the
;; east if east of road X else evacuate to the west)
;;
;; @report direction that is best
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report evacuation-direction
  ifelse (xcor < (first plume_center) - (PLUME_POS_TOLERANCE - PLUME_POS_TOLERANCE * knowledge))
  [
    report DIR_WEST
  ]
  [
    report DIR_EAST
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Local Goal
;; @Turtle
;;
;; Make decisions to reach the local goal stored in local_goal (list of coordinates)
;; Turtle will try to turn toward the intersection stored in local_goal
;; Turtle does not take into account different routes and just makes simple decisions
;; Turtle will set can_change? flag true if it chooses a turn but can turn the opposite direction as well
;; This is used to change the goal if a turn is illegal because of one-way roads
;;
;; @param new_dir the direction the turtle is going (or will be going if turning left)
;;
;; @pre  not empty? local_goal
;; @post turtle will change its turn_goal based on the direction it is going and where it is
;; @post call fix-no-left and fix-one-way
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to local-goal [new_dir]
    
  ;; problem: what if the next spot is the local_goal, but there is a second ultimate goal?
  ;; solution: switch to that new goal NOW
  ;; problem: what if the goals are in a list but they are the SAME
  ;; solution: loop this
  while [length local_goal >= 2 and 
    item 0 curr_goal = item 0 local_goal and item 1 curr_goal = item 1 local_goal]
  [
    set local_goal remove-item 0 local_goal
    set local_goal remove-item 0 local_goal
  ]
  
  ifelse (empty? local_goal)
  [
    update-goal true
  ]
  [
    ;; first, are we going away from the goal?
    ;; first look if we are going away from the goal in the x direction
    ifelse (
      ((item 0 curr_goal) < (item 0 local_goal) and
        new_dir = DIR_WEST) or
      ((item 0 curr_goal) > (item 0 local_goal) and
        new_dir = DIR_EAST)
    )
    [
      ;; need to start turning around
      ifelse (
        ((item 1 curr_goal) <= (item 1 local_goal) and (item 0 curr_goal) < (item 0 local_goal)) or
        ((item 1 curr_goal) >= (item 1 local_goal) and (item 0 curr_goal) > (item 0 local_goal))
      )
      [
        ;; if the turtle is in line with the goal in the y direction, it can go either way
        if ((item 1 curr_goal) = (item 1 local_goal))
        [
          set can_change? true
        ]
        ;; turn right
        set turn_goal 90
      ]
      [
        ;; turn left
        set turn_goal -90
      ]
    ]
    [
      ;; this is the y direction
      ifelse (
        ((item 1 curr_goal) < (item 1 local_goal) and
          new_dir = DIR_SOUTH) or
        ((item 1 curr_goal) > (item 1 local_goal) and
          new_dir = DIR_NORTH)
      )
      [
        ;; need to start turning around
        ifelse (
          ((item 1 curr_goal) > (item 1 local_goal) and (item 0 curr_goal) <= (item 0 local_goal)) or
          ((item 1 curr_goal) < (item 1 local_goal) and (item 0 curr_goal) >= (item 0 local_goal))
        )
        [
          ;; if the turtle is in line with the goal in the x direction, it can go either way
          if ((item 0 curr_goal) = (item 0 local_goal))
          [
            set can_change? true
          ]
          ;; turn right
          set turn_goal 90
        ]
        [
          ;; turn left
          set turn_goal -90
        ]
      ]
      [
        ;; well we are not heading directly away from the goal,
        ;; next we see if we need to turn toward goal, or continue straight
        ifelse (
          ((item 0 curr_goal) = (item 0 local_goal) and (new_dir = DIR_NORTH or new_dir = DIR_SOUTH)) or
          ((item 1 curr_goal) = (item 1 local_goal) and (new_dir = DIR_WEST or new_dir = DIR_EAST))
        )
        [
          ;; This means we are heading directly at goal
          set turn_goal 0
        ]
        [
          ;; Are we at a point where we need to turn or we will go past the goal?
          ifelse (
            (item 0 curr_goal) = (item 0 local_goal) or
            (item 1 curr_goal) = (item 1 local_goal)
            ;; we know they are not heading right at goal, don't need to check dir just yet
          )
          [
            ;; need to turn, pick a direction
            ifelse (
              ((item 0 curr_goal) = (item 0 local_goal) and
                (
                  ((item 1 curr_goal) > (item 1 local_goal) and new_dir = DIR_EAST) or
                  ((item 1 curr_goal) < (item 1 local_goal) and new_dir = DIR_WEST)
                )
              ) or
              ((item 1 curr_goal) = (item 1 local_goal) and
                (
                  ((item 0 curr_goal) > (item 0 local_goal) and new_dir = DIR_SOUTH) or
                  ((item 0 curr_goal) < (item 0 local_goal) and new_dir = DIR_NORTH)
                )
              )
            )
            [
              ;; turn right
              set turn_goal 90
            ]
            [
              ;; turn left
              set turn_goal -90
            ]
          ]
          [
            ;; This means we are heading parallel to the goal,
            ;; not necessarily going to get to it by going straight,
            set turn_goal 0
            
            ;; @todo+ this could add logic to favor evac/artery roads
            ;;        in that case, it must be based on knowledge
            ;;        The logic could just make a turn if the road coming up has multiple lanes
          ]
        ]
      ]
    ]
  ] ;; not empty? local_goal
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Leave Goal
;; @Turtle
;;
;; Make decisions to leave city based on leave_goal
;; Turtle will try to leave the city in the direction of leave_goal
;; Turtle will avoid turning along edges of map and will only actually turn 50% of the time
;; because it should always be able to turn in time
;;
;; @param new_dir the direction the turtle is going (or will be going if turning left)
;;
;; @pre  empty? local_goal and leave? = true and evac? != true
;; @post turtle will change its turn_goal based on the direction it is going
;; @post call fix-one-way
;; @post fix-no-left is not needed because it is built-in in a simpler way
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to leave-goal [new_dir]
  ifelse (random-float 1 < PERCENT_LEAVE_TURN)
  [
    ;; leave_goal is an angle which correlates to heading of car (new_dir)
    ;; subtract-headings will get the angle we need to rotate, should be 0, 90, -90, or 180
    set turn_goal (subtract-headings leave_goal new_dir)
    
    ;; if we have a turn of 180 (turn around) we need to fix it to make two turns
    if (turn_goal = 180)
    [
      ;; if it is a one-way road ahead and the turtle tries to turn the wrong way,
      ;; it can safely turn the correct way
      set can_change? true
      ;; let's favor turning right because it is faster
      ifelse (random-float 1 < PERCENT_LEAVE_RIGHT)
      [
        ;; PERCENT_LEAVE_RIGHT% chance turning right
        set turn_goal 90
      ]
      [
        set turn_goal -90
      ]
    ]
    
    ;; our own no left turn procedure that is easier to deal with for leaving cars
    if ((new_dir = DIR_WEST or new_dir = DIR_EAST) and member? patch-here evac_roads)
    [
      ;; don't allow any changing which could cause illegal turn
      set can_change? false
      
      if (turn_goal = -90)
      [
        ;; illegal left turn, workaround by turning right, after that, the turtle will correct it's path
        set turn_goal 90
      ]
    ]
  ]
  [
    set turn_goal 0
  ]
end

;; check to see if a car leaving the map has reached its destination
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Leave Goal Done?
;; @Turtle
;;
;; Check to see if the Turtle has reached its leave goal in the correct direction
;; Looks to see if the turtle is on the correct edge with the correct heading
;;
;; @report whether or not turtle has reached goal
;; @pre  turtle is leave? = true and empty? local_goal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report leave-goal-done?
  ifelse (leave_goal = heading and
    (
      (heading = DIR_EAST and 
        (pxcor = max-pxcor or
          (speed > 0 and [pxcor] of (patch-ahead 1) = max-pxcor) or
          (speed > 1 and [pxcor] of (patch-ahead (ceiling (abs speed))) = max-pxcor)
          )
        ) or
      (heading = DIR_WEST and 
        (pxcor = min-pxcor or
          (speed > 0 and [pxcor] of (patch-ahead 1) = min-pxcor) or
          (speed > 1 and [pxcor] of (patch-ahead (ceiling (abs speed))) = min-pxcor)
          )
        ) or
      (heading = DIR_NORTH and 
        (pycor = max-pycor or
          (speed > 0 and [pycor] of (patch-ahead 1) = max-pycor) or
          (speed > 1 and [pycor] of (patch-ahead (ceiling (abs speed))) = max-pycor)
          )
        ) or
      (heading = DIR_SOUTH and 
        (pycor = min-pycor or
          (speed > 0 and [pycor] of (patch-ahead 1) = min-pycor) or
          (speed > 1 and [pycor] of (patch-ahead (ceiling (abs speed))) = min-pycor)
          )
        )
      )
    )
  [
    report true
  ]
  [
    report false
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Fix No Left
;; @Turtle
;;
;; Do not allow left turns on evacuation roads (only when actually on the road heading east/west)
;; If attempting this left turn, just turn right so that the normal decision making will correct the route
;;
;; @param new_dir the direction the turtle is going (or will be going if turning left)
;;
;; @pre  local-goal or leave-goal has been called (evacuate has this built-in)
;; @post turtle will make three rights when left turns are not allowed, or turn right if possible instead
;; @note safe to call before or after fix-one-way
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to fix-no-left [new_dir]
  ;; First, correct for no left turns on evac roads
  ;; DOH, check to make sure they are going right or left so they are actually ON the evac road
  if ((new_dir = DIR_WEST or new_dir = DIR_EAST) and turn_goal = -90 and member? patch-here evac_roads)
  [
    ifelse (can_change?)
    [
      set turn_goal 90
      set can_change? false
      ;; that could be illegal
      fix-one-way new_dir
    ]
    [
      ;; illegal left turn, workaround by making three rights
      ;; do this by setting goal to straight and adding an intermediate goal
      ;; that is "two right turns away" from the next intersection straight
      set turn_goal 0
      ;; figure out an intermediate to force the path we want it to take
      let x item 0 curr_goal
      let y item 1 curr_goal
      if (new_dir = DIR_WEST) [set y ((y + 1) mod 8)]
      if (new_dir = DIR_EAST) [set y ((y - 1) mod 8)]
      if (new_dir = DIR_NORTH) [set x ((x + 1) mod 8)]
      if (new_dir = DIR_SOUTH) [set x ((x - 1) mod 8)]
      set local_goal fput y local_goal
      set local_goal fput x local_goal
      
      ;; no need to update with this new goal,
      ;;the car will naturally take the correct course when it reaches the next intersection
    ]
  ]
  ;; don't allow cars on evacuation routes to change to left turns for one-way roads
  if ((new_dir = DIR_WEST or new_dir = DIR_EAST) and turn_goal = 90 and can_change? = true and member? patch-here evac_roads)
  [
    set can_change? false
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Fix One Way
;; @Turtle
;;
;; Do not allow turns the wrong way on to one-way roads by never allowing these illegal turns to be a goal
;; Correct this by turning the opposite if can_change? = true or just going straight
;; Goals will self-correct next time
;;
;; @param new_dir the direction the turtle is going (or will be going if turning left)
;;
;; @pre  local-goal, leave-goal, or evacuate has been called to update goal
;; @post turtle will change its turn_goal if it is making an impossible turn next
;; @note safe to call before or after fix-no-left
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to fix-one-way [new_dir]
  if (turn_goal != 0)
  [
    let temp_row item 1 curr_goal
    let temp_col first curr_goal
    let temp_dir ((new_dir + turn_goal) mod 360)
    ;; Check for one-way road correction
    if (not any? ((item temp_col (item temp_row intersection_matrix)) with [member? temp_dir flow_dir]))
    ;;if (not any? patches with [row = item 1 temp_g and column = item 0 temp_g and member? temp_dir flow_dir])
    [
      ;; The turn won't work because of one-way roads
      
      ;; if the turtle can turn the other way, allow it
      ifelse (can_change?)
      [
        set turn_goal (- turn_goal)
        set can_change? false ;; reset flag for recursion
                              ;; Need to call recursively because above operation is unsafe
        fix-one-way new_dir
      ]
      [
        set turn_goal 0
      ]
    ]
  ]
  ;; whoops, need to reset that
  set can_change? false ;; reset flag
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Find New Goal
;; @Turtle
;;
;; Randomly choose a new goal
;; Some percentage of goals is local, the other is non-local leaving goals
;; This will set a random goal, for leaving goals it will check to make sure
;; the goal is not immediately complete (this is probable with 25% chance)
;;
;; @pre  setup-turtles
;; @post turtle will have some goal set (evacuation goals will never get new goals)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to find-new-goal
  ;; $@DEBUG
  set $goal_change $goal_change + 1
  ;; $@END
  ;; during an alarm, if radios are being used, cars must be sufficiently aware to evacuate
  ifelse (alarm? and will-evacuate? 1)
  [
    set evac? true
    set evac_goal evacuation-direction
    set leave? false
    set local_goal []
  ]
  [
    set evac? false
    
    ifelse (random-float 1 < PERCENT_LEAVING)
    [
      set leave? true
      set local_goal []
      
      ;; may be on edge of map especially if they just had a leave? true goal
      ;; make sure the goal isn't instantly true
      loop
      [
        set leave_goal (random 4) * 90 ;; 0, 90, 180, or 270
        if (not leave-goal-done?)
        [
          stop ;; ********remember, this stops the PROCEDURE, not the loop
        ]
      ]
    ]
    [
      set leave? false
      set local_goal list random NUM_ROADS_X random NUM_ROADS_Y
    ]
  ]
  ;; ***THIS WILL NOT BE REACHED BY CARS WITH LEAVE_GOAL!!!!
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Maybe Evacuate
;; @Turtle
;;
;; Cars with increasing awareness may end up changing to an evacuate goal
;; before completing their current goal at any time 
;; This requires that cars are sufficiently aware and it will happen with a low
;; probability for a given time step
;;
;; @pre  setup-turtles, not evacuating now, radios are on, NOT at the curr_goal intersection
;; @post turtle may now evacuate and will update their turn goal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to maybe-evacuate
  ;; here we divide the chance of evacuating so that this takes longer on average to happen
  if (alarm? and not evac? and will-evacuate? PERCENT_EVAC_TIME_DIV)
  [
    set evac? true
    set evac_goal evacuation-direction
    set leave? false
    set local_goal []
    
    ;; want to start following that goal, here we ASSUME THEY ARE NOT AT CURR_GOAL
    update-goal true
    
    ;; $@DEBUG
    set $goal_change $goal_change + 1
    ;; $@END
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Will Evacuate?
;; @Turtle
;;
;; Calculates if a car will evacuate based on a number of factors and random chance
;;
;; @param div The amount to divide the percentage by (pass 1 normally)
;; @report true if the car will evacuate
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report will-evacuate? [div]
  report
  (
    (awareness >= AWARENESS_THRESHOLD_LOW and
      random-float 1 < PERCENT_HEAR_AFTER * (awareness) / div
      ) or 
    (not radios? and
      random-float 1 < PERCENT_HEAR_AFTER
      )
    )
end

;################################################################################
;; Misc. Car Stuff, less important
;################################################################################

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Set Car Color
;; @Turtle
;; @note GUI only
;;
;; Sets the color based on speed and to differentiate left turners
;;
;; @post turtle color changed
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to set-car-color
  ifelse breed = left_turners
  [
    set color lime
  ]
  [
    ifelse breed = garage_cars
    [
      set color orange
    ]
    [
      ifelse leave? and empty? local_goal
      [
        set color violet
      ]
      [
        ifelse evac?
        [
          set color 14
        ]
        [
          set color blue
        ]
      ]
    ]
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Record Data
;; @Turtle
;;
;; Keep track of the time a turtle spends waiting (speed = 0) and the number of stopped cars
;; Used in analysis
;;
;; @pre  setup-turtles
;; @post wait_time is updated, num_cars_stopped is updated
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to record-data
  ;; wait time and stopped cars
  ifelse (speed = 0)
  [
    set num_cars_stopped num_cars_stopped + 1
    set wait_time wait_time + 1
  ]
  [
    set wait_time 0
  ]
  
  ;; time between intersections
  ifelse ([intersection?] of patch-here)
  [
    set block_time 0
  ]
  [
    set block_time block_time + 1
  ]
end

;################################################################################
;; Traffic Lights
;################################################################################

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Set Signals
;; @Observer
;; @ask controllers (patch agentset)
;;
;; Set up the traffic light color based on the phase of the intersection
;; Sets green, red, and yellow lights
;; Traffic timings are set with east/west roads green first
;;
;; @post traffic light colors are updated with current cycle
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to set-signals
  ;; check current state
  ask controllers
  [
    set state light-state
  ]
  
  ;; red to green horizontal
  ask controllers with [light-state = LIGHT_GR]
  [
    set light_h green
    set light_v red
    set-signal-colors
  ]
  ;; green to yellow horizontal
  ask controllers with [light-state = LIGHT_YR]
  [
    set light_h yellow
    set light_v red
    set-signal-colors
  ]
  ;; all red
  ask controllers with [light-state = LIGHT_RR1]
  [
    set light_h red
    set light_v red
    set-signal-colors
  ]
  ;; yellow to red vertical
  ask controllers with [light-state = LIGHT_RG]
  [
    set light_h red
    set light_v green
    set-signal-colors
  ]
  ;; green to yellow vertical
  ask controllers with [light-state = LIGHT_RY]
  [
    set light_h red
    set light_v yellow
    set-signal-colors
  ]
  ;; all red
  ask controllers with [light-state = LIGHT_RR2]
  [
    set light_h red
    set light_v red
    set-signal-colors
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Light State
;; @Patch : controllers
;;
;; Based on the light timing and current phase, report what state the lights are in
;; This state describes the color of horizontal and vertical lights
;;
;; @report the state of the lights as an enumerated constant
;; @pre  lights have all necessary parameters set
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report light-state
  ifelse (phase >= 0 and phase < green_ticks_h - yellow_ticks_h - TICKS_ALL_RED)
  [
    report LIGHT_GR
  ]
  [
    ifelse (phase >= green_ticks_h - yellow_ticks_h - TICKS_ALL_RED and phase < green_ticks_h - TICKS_ALL_RED)
    [
      report LIGHT_YR
    ]
    [
      ifelse (phase >= green_ticks_h - TICKS_ALL_RED and phase < green_ticks_h)
      [
        report LIGHT_RR1
      ]
      [
        ifelse (phase >= green_ticks_h and phase < green_ticks_h + green_ticks_v - yellow_ticks_v - TICKS_ALL_RED)
        [
          report LIGHT_RG
        ]
        [
          ifelse (phase >= green_ticks_h + green_ticks_v - yellow_ticks_v - TICKS_ALL_RED and phase < green_ticks_h + green_ticks_v - TICKS_ALL_RED)
          [
            report LIGHT_RY
          ]
          [
            ifelse (phase >= green_ticks_h + green_ticks_v - TICKS_ALL_RED)
            [
              report LIGHT_RR2
            ]
            [
              ;; error
              report -1
            ]
          ]
        ]
      ]
    ]
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Set Signals
;; @Patch : controllers
;;
;; Set up the traffic light colors after they have changed
;;
;; @pre  colors have been updated in set-sgnals
;; @post traffic light colors are visually updated
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to set-signal-colors
  let h light_h
  let v light_v
  ask hlights [set pcolor h]
  ask vlights [set pcolor v]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Next Phase
;; @Observer
;; @ask controllers
;;
;; Increment the phase counter by one and modulo it by the total cycle length
;; This is what keeps the lights changing
;;
;; @pre  setup-controllers
;; @post phase is updated for all controllers
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to next-phase
  ;; The phase cycles from 0 to ticks-per-cycle, then starts over.
  ask controllers 
  [
    ;; propogate new light timings when needed
    if (change_state > STATE_STALE or (new_green_h > 0 or new_green_v > 0))
    [
      ;; check to see if the lights need to be changed
      change-light-timing
    ]
    
    set phase (phase + 1) mod (green_ticks_h + green_ticks_v)
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Change Light Timing
;; @Patches (Controllers)
;;
;; change the timing of lights when a new timing is introduced
;; First time this will read new values, the second time it will remove the offset
;;
;; @param start is the new phase to be in, useful when we can change light timings mid-phase
;; @pre  phase is during horizontal green, all red, or will not change with new timing
;; @pre  temp_offset is nonnegative
;; @post timings of lights changed based on new_ variables and temp_offset
;; @post phase will be incremented
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to change-light-timing

  ifelse (change_state > STATE_SYNCING)
  [
    ;; safe to change if lights were all red last phase, wait if they are yellow, and force them yellow from green
    ;; check for lights that were all red
    if (light-state = LIGHT_RR1 or light-state = LIGHT_RR2)
    [
      ;; ready to change
      set change_state STATE_SYNCING
      ;; new timings to be set
      set green_ticks_h new_green_h ;; offset used to get phasing
      set new_green_h 0
      set green_ticks_v new_green_v
      set new_green_v 0
      set yellow_ticks_h new_yellow_h
      set new_yellow_h 0
      set yellow_ticks_v new_yellow_v
      set new_yellow_v 0
      ;; set phase to offset
      set phase temp_offset
      set temp_offset 0
      
      ;; now we check to make sure we aren't setting the lights to yellow (the only problem we could have)
      if (light-state = LIGHT_YR)
      [
        set new_green_v (green_ticks_h - 1 - phase)
        set green_ticks_v (green_ticks_v + new_green_v)
        set phase green_ticks_h - 1
      ]
      ;; if the yellow light is vertical, it's a little trickier because then we wrap around immediately
      if (light-state = LIGHT_RY)
      [
        set new_green_v (green_ticks_h + green_ticks_v - phase + 1)
        set green_ticks_v (green_ticks_v + new_green_v)
        set phase 1
      ]
      
      ;; decrement phase so it can increment
      set phase phase - 1
      
      ;; count changed lights, part of synchronization
      set changed_lights changed_lights + 1
      ;set change_list replace-item row change_list (item row change_list + 1)
    ]
    
    if (change_state = STATE_FRESH)
    [
      ;; keep track of if we changed the lights
      let fix? false
      ;; force green lights to yellow
      while [light-state = LIGHT_GR]
      [
        ;; make it LIGHT_YR by decrementing the green_h time
        set green_ticks_h (green_ticks_h - 1)
        set fix? true
      ]
      while [light-state = LIGHT_RG]
      [
        ;; make it LIGHT_RY by decrementing the green_v time
        set green_ticks_v (green_ticks_v - 1)
        set fix? true
      ]
      ;; if we changed the lights from green to yellow, decrement the phase so that the increment will work
      if (fix?)
      [
        set phase phase - 1 ;; no mod because it must be incremented after this
      ]
      
      set change_state STATE_MID
    ]
  ]
  [
    ;; remove offsets when phase is 0 and all intersections in row have changed
    if (phase = 0 and (change_state = STATE_SYNCING or new_green_h > 0 or new_green_v > 0) and
        changed_lights = NUM_ROADS_X * NUM_ROADS_Y);(item row change_list = NUM_ROADS_X))
    [
      ;; new lights have been set, remove offset
      set green_ticks_h green_ticks_h - new_green_h
      set new_green_h 0
      set green_ticks_v green_ticks_v - new_green_v
      set new_green_v 0
      ;; remove the flag
      set change_state STATE_STALE
    ]
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Sync Timing
;; @Patches (Controllers)
;;
;; synchronize changed lights in order to keep the phasing in time even if the lights are different
;;
;; @pre  this light has changed
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to sync-timing
  if (change_state = STATE_SYNCING)
  [
    ;; make the current phase longer while we wait for the other lights
    set green_ticks_h green_ticks_h + 1
    set new_green_h new_green_h + 1
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Read New Signal Timing
;; @Observer
;; @ask: Patches (controllers)
;;
;; Change the light timings to evacuation mode
;;
;; @pre  alarm has sounded
;; @post new evacuation timing is read
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to read-new-signal-timing
  ifelse (file-exists? evac_file)
  [
    file-open evac_file
    foreach sort controllers
    [
      ask ?
      [
        set temp_offset file-read
        set new_green_h file-read
        set new_yellow_h file-read
        set new_green_v file-read
        set new_yellow_v file-read
        ;; fix offset so it does not get large
        set temp_offset temp_offset ;mod (new_green_h + new_green_v)
        set change_state STATE_FRESH
      ]
    ]
    file-close
    ;; sort the lights in order right to left top to bottom
    foreach sort-by [[pycor] of ?1 > [pycor] of ?2 or
      (
        [pycor] of ?1 = [pycor] of ?2 and
        [pxcor] of ?1 > [pxcor] of ?2
      ) ] controllers
    [
      ask ?
      [
        adjust-offset
      ]
    ]
    set changed_lights 0
    ;; keep track of changes by row
    ;set change_list (list)
    ;while [length change_list <= NUM_ROADS_Y]
    ;[
    ;  set change_list fput 0 change_list
    ;]
  ]
  [
    set error_text "***ERROR: Could not open file for evacuation lights"
    print error_text
    set EXCEPTION? true
    report-results true
    stop
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Adjust Offset
;; @Patches : Controllers
;;
;; Adjust the offset of lights based on their variable distance apart
;; Makes assumptions that the offsets are set from right to left
;; Experimental results show that cars driving average speed (1) will cover a distance "d"
;; from a stop in (d + 2) time
;;
;; @pre  Light timings are uniform from right to left
;; @pre  Called in sorted order (top to bottom, right to left) so that light to the left is unchanged
;; @post temp_offset is set based on road distance
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to adjust-offset
  if (temp_offset != 0)
  [
    let change 0
    ;; the last road in the column doesn't matter that much
    if (column != 0)
    [
      ;; get difference between this offset and next
      let individual_amount (abs (temp_offset) - [temp_offset] of one-of controllers with
        [
          row = [row] of myself and
          column = ([column] of myself - 1) mod NUM_ROADS_X 
        ])
      ;; the distance between this road and next based on position of controllers
      let dist_to_road (abs (pxcor) - [pxcor] of one-of controllers with
        [
          row = [row] of myself and
          column = ([column] of myself - 1) mod NUM_ROADS_X 
        ])
      ;; the percentage of this difference from the average, factoring in the 2 units of time to accelerate
      let dist_percent (dist_to_road + 2) / floor(GRID_X_SIZE + 2)
      ;; how much the offset should change
      set change (individual_amount * dist_percent) - individual_amount
      ;; actually change the offset
    ]
    set temp_offset (floor (temp_offset + change)) mod (new_green_h + new_green_v)
  ]
end

;################################################################################
;; Results Reporting
;################################################################################

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Evacuation Done?
;; @Observer
;;
;; Counts cars and looks at the stuck cars to determine if the evacuation is over
;;
;; @report boolean value true if the run can be stopped
;;
;; @pre  model running
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report evacuation-done?
  let result false
  ;; why should this check (which could vary based on exit condition) modify random numbers at every tick?
  with-local-randomness
  [
    ;let disabled_count count (turtle-set cars left_turners) with [incapacitated?])
    let remaining_alive count (turtle-set cars left_turners garage_cars off_map_cars) with [not incapacitated?]
    if-else (remaining_alive > 0 and remaining_alive < CAR_TOTAL * (1 - PERCENT_REQUIRED_TO_EVAC))
    [
      ;; are there any cars that have not been waiting for less than 2 minutes?
      set result not any? (turtle-set cars left_turners garage_cars) with [not incapacitated? and wait_time < 240]
    ]
    [
      ;; either there are no cars or there are lots of cars so...
      set result remaining_alive = 0
    ]
  ]
  report result
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Report Results
;; @Observer
;;
;; Opens a file and reports the results of the simulation run
;; for use by external application
;; File name is set in "out_file"
;;
;; @param fail? this flag set to true means that the model failed in some way
;;
;; @pre  go has been run
;; @post file out_file contains results appended to previous contents
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to report-results [fail?]
  if (EXCEPTION? and not logged?)
  [
    ;; log the error for quick find
    log-error
  ]
  
  let temp_cars no-turtles
  if (not fail?)
  [
    ;; look at remaining cars in garages and simulate their exposure
    ask garages
    [
      if (car_amount > 0)
      [
        ;; loop through car_amount and create the cars then destroy them
        repeat car_amount
        [
          ;; get a new car and set it up
          let c new-car-hatch
          ;; store for deletion
          set temp_cars (turtle-set temp_cars c)
          ask c
          [
            ;show-turtle
            set breed garage_cars
            setup-car
            ;; update this cars exposure to the level of the garage
            set exposure [exposure] of myself
            check-aegl ;; this will affect results
            ;; $@LOGVERBOSE
            show (word "Garage car calculated with AEGL-" aeglevel)
            ;; $@END
          ]
        ]
      ]
    ]
  ]
  ;; open the file
  file-open out_file
  
  ;; format:
  ;; line 1: {status code} v:{model version}, time:{timestamp} seconds:{runtime in seconds}
  ;; line 2: {status code number}
  ;; line 3: AEGL-2 {AEGL-2 count}
  ;; line 4: AEGL-3 {AEGL-3 count}
  ;; line 5: TICKS  {ticks at end|default TICK_LIMIT}
  ;; line 6: CARCNT {non-disabled cars remaining|default 0}
  ;; line 7: GARAGE {cars from garages we used in calculations}
  ;; line 8: END seed:{integer seed used for run}
  
  ;; values read starting at character 7 (0 indexed count)
  
  ifelse (fail?)
  [
    ;; just in case, but this should have been done already
    set EXCEPTION? true
    ;; something went wrong
    print "***ERROR OCCURRED!***"
    file-print (word "FAIL v:" VERSION ", time:" date-and-time " seconds:" timer)
    file-print -1
    file-print (word "AEGL-2 " max (list ((count (turtle-set cars left_turners garage_cars) with [aeglevel = 2]) * 1000) 10000))
    file-print (word "AEGL-3 " max (list ((count (turtle-set cars left_turners garage_cars) with [aeglevel = 3]) * 1000) 10000))
    file-print (word "TICKS  " max (list (ticks * 1000) 10000))
    file-print (word "CARCNT " max (list ((count (turtle-set cars left_turners garage_cars off_map_cars) with [not incapacitated?]) * 1000) 10000))
    file-print (word "GARAGE " max (list ((count temp_cars) * 1000) 10000))
    file-print (word "RUNNUM " run_num)
  ]
  [
    ifelse (ticks < TICK_LIMIT)
    [
      ;; all cars evacuated or disabled by time limit
      file-print (word "COMPLETE v:" VERSION ", time:" date-and-time " seconds:" timer)
      file-print 2
    ]
    [
      ;; hit time limit
      file-print (word "NORMAL v:" VERSION ", time:" date-and-time " seconds:" timer)
      file-print 1
    ]
    ;; common data
    file-print (word "AEGL-2 " count (turtle-set cars left_turners garage_cars) with [aeglevel = 2])
    file-print (word "AEGL-3 " count (turtle-set cars left_turners garage_cars) with [aeglevel = 3])
    file-print (word "TICKS  " ticks)
    file-print (word "CARCNT " count (turtle-set cars left_turners garage_cars off_map_cars) with [not incapacitated?])
    file-print (word "GARAGE " count temp_cars)
    file-print (word "RUNNUM " run_num)
  ]
  ;; footer to signal end of data
  file-print (word "END seed:" seed-int ", " seed_decimal " file:" evac_file)
  
  file-close
  
  ;; remove the cars made just for calculation
  ask temp_cars
  [
    recycle
  ]
  
  set done? true
  ;; send to standard output so we know when it finished
  print "***END**************************************************************************"
  print date-and-time 
  print "********************************************************************************"
  
  ;; make sure model is not continued again if it is done
  if (file-exists? resume_file)
  [
    file-delete resume_file
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Log Error
;; @Observer
;;
;; Opens error log when an exception is thrown
;; useful for debugging runs with multiple instances running
;;
;; @pre  EXCEPTION? true
;; @post error is logged and "logged?" is set to true
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to log-error
  if (not logged? and EXCEPTION?)
  [
    carefully
    [
      file-open log_file
      file-print "<entry>"
      file-print (word "    <run>" run_num "</run>")
      file-print (word "    <datetime>" date-and-time "</datetime>")
      file-print (word "    <runtime>" timer "</runtime>")
      file-print (word "    <details>" error_text "</details>")
      file-print "</entry>\n"
      file-close
    ]
    [
      print "***FATAL ERROR LOGGING ERROR****************************************************"
    ]
  ]
  set logged? true
end

;################################################################################
;; State Save and Resume
;################################################################################

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Load State
;; @Observer
;;
;; If the state file exists, opens the file using import-world
;;
;; @pre  setup and reset have been run to guarantee accuracy and no exceptions were thrown
;; @post state of model resumes if it had been saved, clears old exceptions from over time limit
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to load-state
  if (file-exists? resume_file and not EXCEPTION?)
  [
    print (word "Loading previous state " date-and-time)
    import-world resume_file
    ;; So we left off in PROBABLY an error state (exception thrown).  Now we reset that
    set EXCEPTION? false
    set error_text ""
    set logged? false
    print (word "Loaded previous state " date-and-time)
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Save State
;; @Observer
;;
;; If the state file exists, delete it
;; Store the current state of the model using export-world
;;
;; @pre  Model has finished a go loop
;; @post state of model is stored and model must be restarted to complete
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to save-state
  if (file-exists? resume_file)
  [
    file-delete resume_file
  ]
  print (word "Saving state file" date-and-time)
  carefully
  [
    export-world resume_file
  ]
  [
    set error_text (word "***ERROR: Failed to save state file at: " resume_file)
    print error_text
    set EXCEPTION? true
    log-error
  ]
  print "***END**************************************************************************"
  print date-and-time 
  print "********************************************************************************"
end

;################################################################################
;; Debugging
;################################################################################

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Goal Num
;; @Turtle
;;
;; Turn the ultimate goal of a turtle into an integer
;; (think octal representation of x: [0, 7] y: [0, 7] = 0xy)
;;
;; @report integer representing ultimate goal or -1
;; @pre  setup-turtles, find-new-goal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to-report goal-num
  if (not empty? local_goal)
  [
    report (item 0 local_goal) * 8 + (item 1 local_goal)
  ]
  report -1
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Show Gas
;; @Observer
;;
;; Shows visual indication of plume concentration to debugging and output
;;
;; @post colors are all wrong but will be reset next time "go" is called (other functions might choke)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to show-gas
  with-local-randomness
  [
    set colors_wrong? true
    ask patches with [pcolor = COLOR_NOT_ROAD]
    [
      let amount concentration
      set pcolor 19.9 - (amount / 100) ;; red is more
    ]
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Test Distance
;; @Observer
;; @ask: Cars
;;
;; A test module to see if linear-distance procedure is working
;;
;; @pre  setup
;; @post sends a message to the output if any of the linear-distance values
;;       differ from the builtin distance
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to $test-dist
  ask cars
  [
    let c find-nearest-car safe-distance heading patch-here
    if (c != nobody and
      (distance c != linear-distance c heading)
      )
    [
      show (word "**Distance is incorrect! distance:" distance c ", linear:" linear-distance c heading)
    ]
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Test Light Offset
;; @Observer
;; @ask: Patches (controllers)
;; @depreciated
;;
;; A test module to see if light offsets are being set correctly
;;
;; @param file the file specifying the offsets
;; @pre  lights have changed due to evacuation or the file is the original file
;; @post sends a message to the output with success or what the problem is
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to #test-light-offset [file]
  ifelse (any? controllers with [change_state != STATE_STALE])
  [
    print "Not all controllers have finished changing light timing, check later"
  ]
  [
    let junk 0
    let r 0
    let actual_time (list)
    let expected_time (list)
    let bug? false
    file-open file
    foreach sort controllers
    [
      ask ?
      [
        ;; each row resets
        if (row != r)
        [
          set r row
          set actual_time (list)
          set expected_time (list)
        ]
        ;; read the values
        set expected_time lput (file-read mod (green_ticks_h + green_ticks_v)) expected_time
        set actual_time lput phase actual_time
        set junk file-read
        set junk file-read
        set junk file-read
        set junk file-read
        ;; end of the row, do calculations
        if (column = NUM_ROADS_X - 1)
        [
          let i 0
          while [i < length actual_time - 1]
          [
            let ex_diff #time-diff (item (i + 1) expected_time) (item i expected_time) (green_ticks_h + green_ticks_v)
            let ac_diff #time-diff (item (i + 1) actual_time) (item i actual_time) (green_ticks_h + green_ticks_v)
            if (ex_diff != ac_diff)
            [
              print (word "Difference found at row:" row " col:" i "," (i + 1) ", should be:" ex_diff " but was:" ac_diff)
              set bug? true
            ]
            set i i + 1
          ]
        ]
      ]
    ]
    file-close
    if (not bug?)
    [
      print "Light timing is correct"
    ]
  ]
end

to-report #time-diff [t1 t2 max_val]
  ifelse (t2 >= t1)
  [
    report t2 - t1
  ]
  [
    report max_val - t1 + t2
  ]
end

;################################################################################
;; Testing Environment
;################################################################################

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Setup Test
;; @Observer
;;
;; Sets up a testing environment
;;
;; @param test_num The specific test to setup after initial boilerplate
;; @post ready to run test and verify results
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to *setup-test [test_num]
  ;; for logging purposes
  print "***TEST*SETUP*******************************************************************"
  print date-and-time
  print "********************************************************************************"
  
  set EXCEPTION? false
  set error_text ""
  ;; start timer here
  reset-timer
  reset-ticks
  ;; only clear special things here that we wouldn't clear in reset
  clear-patches
  clear-turtles
  clear-output
  clear-all-plots
  clear-drawing
  
  ;; use a random seed based on a decimal number from -1 to 1
  random-seed seed-int
  
  setup-globals

  ;; First we ask the patches to draw themselves and set up a few variables
  ;; don't affect the random number generator with this constant setup
  with-local-randomness
  [
    setup-patches
  ]
  
  set-default-shape turtles "car"
  set-default-shape garages "house"

  *change-testcase test_num
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Change Test Case
;; @Observer
;;
;; Resets to a new test case, while keeping the map building stuff
;;
;; @param test_num The specific test to setup after initial boilerplate
;; @post ready to run test and verify results
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to *change-testcase [test_num]
  print (word "***TEST*CASE*" test_num "******************************************************************")
  print date-and-time
  print "********************************************************************************"
  
  reset-ticks
  clear-turtles
  clear-all-plots
  clear-drawing
  
  ;; use a random seed based on a decimal number from -1 to 1
  random-seed seed-int
  
  setup-globals

  with-local-randomness
  [
    ;; set up the initial light phases
    setup-controllers
    ;; reset the concentration
    ask patches
    [
      set concentration 0
    ]
  ]
  
  ;; setup the given test case based on this numbering
  ifelse (test_num = 0) [ *testcase-stop-for-single-car ][
  ifelse (test_num = 1) [ *testcase-fast-car-follow-slow ][
  ifelse (test_num = 2) [ *testcase-car-group-approaching-red ][
  ifelse (test_num = 3) [  ][
  ]]]]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Code injected after "go" for test cases
;; @Observer
;;
;; inject code after running go so specific test case can break some rules of model
;;
;; @pre already did go
;; @param test_num The specific test that is running
;; @post changes based on what is needed for test
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to *post-go-test [test_num]
  ifelse (test_num = 0) [ ][
  ifelse (test_num = 1) [ *testcase-fast-car-follow-slow-go ][
  ifelse (test_num = 2) [ *testcase-car-group-approaching-red-go ][
  ifelse (test_num = 3) [ ][
  ]]]]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Setup Testcase 1
;; @Observer
;;
;; Creates one car which will never move, and makes sure a car behind it will stop
;;
;; @post ready to run test and verify results
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to *testcase-stop-for-single-car
  ;; override some globals
  set garage? false
  set emergency? false
  set plume? false
  
  let stopped_car nobody
  let moving_car nobody
  
  ;; moving car
  ask patch -108 88
  [
    ;; make sure this is where we want to do the test
    if (pcolor = COLOR_NOT_ROAD or first flow_dir != DIR_WEST)
    [
      set error_text "***ERROR: Road layout has changed, update test"
      print error_text
      set EXCEPTION? true
      log-error
      stop
    ]
    sprout-cars 1
    [
      set moving_car self
      setup-car
      
      set my_speed_limit 2
      set leave? true
      set leave_goal DIR_WEST
      set local_goal []
      update-goal true
      
      ;; $@GUI
      set-car-color
      ;; $@END
      record-data
    ]
  ]
  
  ;; stopped car
  ask patch -134 88
  [
    ;; make sure this is where we want to do the test
    if (pcolor = COLOR_NOT_ROAD or first flow_dir != DIR_WEST)
    [
      set error_text "***ERROR: Road layout has changed, update test"
      print error_text
      set EXCEPTION? true
      log-error
      stop
    ]
    sprout-cars 1
    [
      set stopped_car self
      setup-car
      
      set incapacitated? true
      
      ;; $@GUI
      set-car-color
      ;; $@END
      record-data
    ]
  ]
  
  inspect moving_car
  inspect stopped_car
  
  set CAR_TOTAL 2
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Setup Testcase 2
;; @Observer
;;
;; Creates one car going min speed, followed by one car going max, do they collide?
;;
;; @post ready to run test and verify results
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to *testcase-fast-car-follow-slow
  ;; override some globals
  set garage? false
  set emergency? false
  set plume? false
  
  let slow_car nobody
  let fast_car nobody
  
  ;; fast car
  ask patch -108 88
  [
    ;; make sure this is where we want to do the test
    if (pcolor = COLOR_NOT_ROAD or first flow_dir != DIR_WEST)
    [
      set error_text "***ERROR: Road layout has changed, update test"
      print error_text
      set EXCEPTION? true
      log-error
      stop
    ]
    sprout-cars 1
    [
      set fast_car self
      setup-car
      
      set my_speed_limit 2
      set leave? true
      set leave_goal DIR_WEST
      set local_goal []
      update-goal true
      
      ;; $@GUI
      set-car-color
      ;; $@END
      record-data
    ]
  ]
  
  ;; slow car
  ask patch -134 88
  [
    ;; make sure this is where we want to do the test
    if (pcolor = COLOR_NOT_ROAD or first flow_dir != DIR_WEST)
    [
      set error_text "***ERROR: Road layout has changed, update test"
      print error_text
      set EXCEPTION? true
      log-error
      stop
    ]
    sprout-cars 1
    [
      set slow_car self
      setup-car
      
      set my_speed_limit 0.5
      set leave? true
      set leave_goal DIR_WEST
      set local_goal []
      update-goal true
      
      ;; $@GUI
      set-car-color
      ;; $@END
      record-data
    ]
  ]
  
  ;; make all lights green in path
  ask controllers with [pycor = 88]
  [
    set phase 0
  ]
  
  inspect fast_car
  inspect slow_car
  
  set CAR_TOTAL 2
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Go Testcase 2
;; @Observer
;;
;; Don't allow either car to turn
;;
;; @post turn_goals set to 0
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to *testcase-fast-car-follow-slow-go
  ask cars
  [
    set turn_goal 0
  ]
  
  ;; make all lights green in path
  ask controllers with [pycor = 88]
  [
    set phase 0
  ]
  
  let first_car one-of cars with [my_speed_limit = 0.5]
  let second_car one-of cars with [my_speed_limit = 2]
  
  if ([xcor] of second_car < [xcor] of first_car and
    [xcor] of first_car - [xcor] of second_car < 100)
  [
    print "ERROR: car ghost bug!"
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Setup Testcase 3
;; @Observer
;;
;; Group of cars going to red light should not crash
;;
;; @post ready to run test and verify results
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to *testcase-car-group-approaching-red
  ;; override some globals
  set garage? false
  set emergency? false
  set plume? false
  
  ;; need about 40 cars
  ;; start at [-44, 65]
  foreach [65 64 63]
  [
    let y ?
    foreach [-44 -42 -40 -38 -36 -30 -26 -24 -22 -21 -20]
    [
      ask patch ? y
      [
        ;; make sure this is where we want to do the test
        if (pcolor = COLOR_NOT_ROAD or first flow_dir != DIR_WEST)
        [
          set error_text "***ERROR: Road layout has changed, update test"
          print error_text
          set EXCEPTION? true
          log-error
          stop
        ]
        sprout-cars 1
        [
          setup-car
          
          set leave? true
          set leave_goal DIR_WEST
          set local_goal []
          update-goal true
          
          ;; $@GUI
          set-car-color
          ;; $@END
          record-data
        ]
      ]
    ]
  ]
  
  ;; all lights should be red
  ask controllers with [pycor = 65]
  [
    set green_ticks_h 40
    set green_ticks_v 40
    set phase 60
  ]
  
  inspect patch -49 65
  
  set CAR_TOTAL 2
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Go Testcase 3
;; @Observer
;;
;; Keep specific light red, no turning
;;
;; @post turn_goals set to 0
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to *testcase-car-group-approaching-red-go
  ask cars
  [
    set turn_goal 0
  ]
  
  ;; make all lights red in path
  ask controllers with [pycor = 65]
  [
    set phase 60
  ]
  
;  if ([xcor] of second_car < [xcor] of first_car and
;    [xcor] of first_car - [xcor] of second_car < 100)
;  [
;    print "ERROR: car ghost bug!"
;  ]
end

; *** NetLogo 4.0.3 Model Copyright Notice ***
;
; This model was created as part of the projects:
; PARTICIPATORY SIMULATIONS: NETWORK-BASED DESIGN FOR SYSTEMS LEARNING IN
; CLASSROOMS and/or INTEGRATED SIMULATION AND MODELING ENVIRONMENT.
; The project gratefully acknowledges the support of the
; National Science Foundation (REPP & ROLE programs) --
; grant numbers REC #9814682 and REC-0126227.
;
; Copyright 2003 by Uri Wilensky.  All rights reserved.
;
; Permission to use, modify or redistribute this model is hereby granted,
; provided that both of the following requirements are followed:
; a) this copyright notice is included.
; b) this model will not be redistributed for profit without permission
;    from Uri Wilensky.
; Contact Uri Wilensky for appropriate licenses for redistribution for
; profit.
;
; To refer to this model in academic publications, please use:
; Wilensky, U. (2003).  NetLogo Traffic Grid model.
; http://ccl.northwestern.edu/netlogo/models/TrafficGrid.
; Center for Connected Learning and Computer-Based Modeling,
; Northwestern University, Evanston, IL.
;
; In other publications, please use:
; Copyright 2003 Uri Wilensky.  All rights reserved.
; See http://ccl.northwestern.edu/netlogo/models/TrafficGrid
; for terms of use.
;
; *** End of NetLogo 4.0.3 Model Copyright Notice ***
@#$#@#$#@
GRAPHICS-WINDOW
504
10
1477
644
160
100
3.0
1
8
1
1
1
0
1
1
1
-160
160
-100
100
1
1
1
ticks
30.0

PLOT
10
920
228
1084
Average Wait Time of Cars
Time
Average Wait
0.0
100.0
0.0
5.0
true
false
"" ""
PENS
"default" 1.0 0 -2674135 true "" "let agents (turtle-set cars left_turners garage_cars)\nif (any? agents)\n[\n  plot mean [ wait_time ] of agents\n]"

PLOT
245
750
461
915
Average Speed of Cars
Time
Average Speed
0.0
100.0
0.0
1.0
true
false
"" ""
PENS
"default" 1.0 0 -2674135 true "" "let agents (turtle-set cars left_turners garage_cars)\nif (any? agents)\n[\n  plot mean [ speed ] of agents\n]"

PLOT
15
750
229
914
Stopped Cars
Time
Stopped Cars
0.0
100.0
0.0
100.0
true
false
"set-plot-y-range 0 NUM_CARS" ""
PENS
"default" 1.0 0 -2674135 true "" "plot num_cars_stopped"

BUTTON
365
10
429
43
Go
go\n;if-else (ticks < 1000)\n;[go] [stop]\n;if-else (accident_count = 0)\n;[go] [stop]
T
1
T
OBSERVER
NIL
G
NIL
NIL
1

BUTTON
10
10
113
43
Setup (seed)
;random-seed 1231231231987\nsetup
NIL
1
T
OBSERVER
NIL
Z
NIL
NIL
1

BUTTON
435
10
498
43
Step
go
NIL
1
T
OBSERVER
NIL
S
NIL
NIL
1

MONITOR
300
210
395
255
Cars off Map
count off_map_cars
17
1
11

MONITOR
295
165
396
210
Cars Evacuated
cars_safe
17
1
11

MONITOR
315
120
397
165
Cars on Map
count (turtle-set cars left_turners)
17
1
11

BUTTON
120
10
184
43
Reset
reset
NIL
1
T
OBSERVER
NIL
R
NIL
NIL
1

SWITCH
10
50
121
83
emergency?
emergency?
0
1
-1000

PLOT
10
440
250
590
Total Cars Unevacuated
Time
Cars
0.0
100.0
0.0
1000.0
true
false
"set-plot-y-range 0 NUM_CARS + (sum [car_amount] of garages)" ""
PENS
"default" 1.0 0 -16777216 true "" "plot count (turtle-set cars left_turners off_map_cars garage_cars) + (sum [car_amount] of garages)"

MONITOR
400
135
496
180
Max Wait Time
max [wait_time] of turtles
1
1
11

PLOT
255
440
495
590
Visible Cars
Time
Cars
0.0
100.0
0.0
1000.0
true
false
"set-plot-y-range 0 NUM_CARS + (sum [car_amount] of garages)" ""
PENS
"total" 1.0 0 -16777216 false "" "plot count (turtle-set cars left_turners garage_cars)"

PLOT
255
595
495
745
Cars Off Map
Time
Cars
0.0
100.0
0.0
1000.0
true
false
"" ""
PENS
"default" 1.0 0 -2674135 true "" "plot count off_map_cars"

PLOT
240
1075
440
1225
Speeds of Cars
Speed
Cars
0.0
2.0
0.0
500.0
true
false
"" ""
PENS
"default" 0.1 1 -16777216 true "" "histogram [speed] of (turtle-set cars left_turners garage_cars)"

PLOT
240
920
440
1070
Wait Times of Cars
Wait Time
Cars
0.0
10.0
0.0
10.0
true
false
"" "let agents (turtle-set cars left_turners garage_cars)\nifelse (any? agents)\n[\n  set-plot-x-range 0 max [wait_time] of agents + 10\n]\n[\n  set-plot-x-range 0 10\n]"
PENS
"default" 10.0 1 -16777216 true "" "let agents (turtle-set cars left_turners garage_cars)\nif (any? agents)\n[\n  histogram [wait_time] of agents\n]"

SWITCH
10
1100
113
1133
#is_gui?
#is_gui?
0
1
-1000

TEXTBOX
45
1105
165
1131
Do Not Turn this off!!!!
11
0.0
0

MONITOR
400
225
495
270
two to a space
count patches with [count cars-here > 1]
17
1
11

SLIDER
10
260
182
293
NUM_CARS
NUM_CARS
0
6000
5000
100
1
cars
HORIZONTAL

SLIDER
10
295
190
328
TICKS_TO_ALARM
TICKS_TO_ALARM
0
1500
150
1
1
ticks
HORIZONTAL

INPUTBOX
15
335
170
395
*who
4328
1
0
Number

BUTTON
15
400
75
433
Inspect
inspect turtle *who
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
80
400
143
433
draw
ask turtle *who [pd]
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
150
400
213
433
no draw
ask turtle *who [pu]
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
205
325
275
358
Freeze
export-world user-new-file
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
200
365
275
398
Restore
import-world user-file
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
565
1260
667
1293
Start Profiling
;; $@PROFILE\nprofiler:start\n;; $@END
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
675
1260
777
1293
Stop Profiling
;; $@PROFILE\nprofiler:stop\n;; $@END
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
785
1260
882
1293
Print Profiler
;; $@PROFILE\noutput-print profiler:report\n;; $@END
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

OUTPUT
890
1080
1465
1545
12

INPUTBOX
210
10
362
70
seed_decimal
0.3288789
1
0
Number

MONITOR
300
255
397
300
Cars in Garages
(sum [car_amount] of garages)
17
1
11

PLOT
10
595
250
745
Cars in Garages
Time
Cars
0.0
100.0
0.0
1500.0
true
false
"set-plot-y-range 0 (max (list 1 (sum [car_amount] of garages)))" ""
PENS
"default" 1.0 0 -16777216 true "" "plot (sum [car_amount] of garages)"

BUTTON
345
85
427
118
Show Gas
show-gas
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
250
85
347
118
RemoveGas
reset-patch-colors\nask (turtle-set cars left_turners)\n[\n  ;set label \"\"\n]
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SWITCH
120
85
223
118
garage?
garage?
0
1
-1000

BUTTON
675
1300
777
1333
Reset Profiler
;; $@PROFILE\nprofiler:reset\n;; $@END
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
785
1300
847
1333
Clear
clear-output
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

PLOT
715
655
1040
840
AEGL
Time
Affected
0.0
100.0
0.0
100.0
true
true
"" ""
PENS
"AEGL1" 1.0 0 -1184463 true "" "plot count (turtle-set cars left_turners garage_cars) with [aeglevel = 1]"
"AEGL2" 1.0 0 -955883 true "" "plot count (turtle-set cars left_turners garage_cars) with [aeglevel = 2]"
"AEGL3" 1.0 0 -2674135 true "" "plot count (turtle-set cars left_turners garage_cars) with [aeglevel = 3]"

SWITCH
10
85
113
118
plume?
plume?
0
1
-1000

BUTTON
365
45
452
78
Go/Export
go\nif (ticks > 0 and ticks mod 600 = 0)\n[\nset-patch-size 8\nexport-view (word ticks \" ticks.png\")\nshow-gas\nexport-view (word ticks \" plume.png\")\nset-patch-size 3\nreset-patch-colors\n]
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

MONITOR
400
270
492
315
patch crowded
count patches with [count cars-here > 4]
17
1
11

MONITOR
335
300
397
345
remaining
count (turtle-set cars left_turners garage_cars) with [not incapacitated? and wait_time < 240]
17
1
11

PLOT
1050
655
1400
840
Awareness/Knowledge
Time
Level
0.0
10.0
0.0
1.0
true
true
"" ""
PENS
"aware" 1.0 0 -10899396 true "" "let agents (turtle-set cars left_turners garage_cars)\nif (any? agents)\n[\n  plot mean [awareness] of agents\n]"
"know" 1.0 0 -13345367 true "" "let agents (turtle-set cars left_turners garage_cars)\nif (any? agents)\n[\n  plot mean [knowledge] of agents\n]"
"urgency" 1.0 0 -955883 true "" "let agents (turtle-set cars left_turners garage_cars)\nif (any? agents)\n[\n  plot mean [sense_of_urgency] of agents\n]"

PLOT
695
865
895
1015
Awareness Distribution
Awareness
Cars
-0.01
1.0
0.0
10.0
true
false
"" ""
PENS
"default" 0.1 1 -16777216 true "" "histogram [awareness] of (turtle-set cars left_turners)"

PLOT
910
865
1110
1015
Knowledge Distribution
Knowledge
Cars
0.0
1.0
0.0
10.0
true
false
"" ""
PENS
"default" 0.1 1 -16777216 true "" "histogram [knowledge] of (turtle-set cars left_turners)"

PLOT
1120
865
1320
1015
Evacuation Direction
direction
cars
0.0
360.0
0.0
10.0
true
false
"" ""
PENS
"default" 90.0 1 -16777216 true "" "histogram [evac_goal] of ((turtle-set cars left_turners) with [evac?])"

SWITCH
120
120
235
153
change_evac_goal_allowed?
change_evac_goal_allowed?
0
1
-1000

BUTTON
45
1235
147
1268
Setup Test x
*setup-test test_selection
NIL
1
T
OBSERVER
NIL
T
NIL
NIL
1

INPUTBOX
155
1235
307
1295
test_selection
1
1
0
Number

BUTTON
45
1275
117
1308
Go Test
go\n*post-go-test test_selection
NIL
1
T
OBSERVER
NIL
X
NIL
NIL
1

BUTTON
45
1200
137
1233
Reset Test
*change-testcase test_selection
NIL
1
T
OBSERVER
NIL
C
NIL
NIL
1

SWITCH
10
120
113
153
radios?
radios?
0
1
-1000

SWITCH
120
155
223
188
signs?
signs?
0
1
-1000

PLOT
490
865
690
1015
Evacuating
Time
Cars
0.0
10.0
0.0
10.0
true
false
"" ""
PENS
"default" 1.0 0 -16777216 true "" "plot count (turtle-set cars left_turners garage_cars) with [evac?]"

PLOT
505
1025
705
1175
Radios
time
cars
0.0
10.0
0.0
10.0
true
false
"" ""
PENS
"default" 1.0 0 -16777216 true "" "plot count (turtle-set cars left_turners garage_cars) with [radio?]"

PLOT
710
1025
910
1175
See Sign
time
cars
0.0
10.0
0.0
10.0
true
false
"" ""
PENS
"default" 1.0 0 -16777216 true "" "plot count (turtle-set cars left_turners garage_cars) with [see_sign > SIGN_TIME]"

MONITOR
400
180
487
225
Max sign time
max [see_sign] of turtles
17
1
11

PLOT
920
1025
1120
1175
Sense of Urgency Distribution
Stress
cars
0.0
1.0
0.0
10.0
true
false
"" ""
PENS
"default" 0.1 1 -16777216 true "" "histogram [sense_of_urgency] of (turtle-set cars left_turners)"

SWITCH
10
155
113
188
stress?
stress?
0
1
-1000

PLOT
480
685
710
835
Road Volume
Time
Cars
0.0
10.0
0.0
10.0
true
true
"" ""
PENS
"evac" 1.0 0 -10899396 true "" "plot (count cars-on evac_roads) + (count left_turners-on evac_roads)"
"other" 1.0 0 -2674135 true "" "plot (count cars-on roads) + (count left_turners-on roads)"

SWITCH
125
50
242
83
knowledge?
knowledge?
0
1
-1000

SWITCH
10
190
192
223
plume_random_offset?
plume_random_offset?
0
1
-1000

SWITCH
175
190
287
223
accidents?
accidents?
0
1
-1000

MONITOR
400
315
462
360
Accidents
accident_count
17
1
11

SWITCH
10
225
112
258
u_turns?
u_turns?
0
1
-1000

@#$#@#$#@
## WHAT IS IT?

## HOW IT WORKS

## HOW TO USE IT

Buttons:


Sliders:


Switches:


Plots:



## THINGS TO NOTICE

## THINGS TO TRY

## EXTENDING THE MODEL

## NETLOGO FEATURES

## RELATED MODELS

Traffic Basic simulates the flow of a single lane of traffic in one direction  
Traffic 2 Lanes adds a second lane of traffic  
Traffic Intersection simulates a single intersection

The HubNet activity Gridlock has very similar functionality but allows a group of users to control the cars in a participatory fashion.

## CREDITS AND REFERENCES

To refer to this model in academic publications, please use:  Wilensky, U. (2003).  NetLogo Traffic Grid model.  http://ccl.northwestern.edu/netlogo/models/TrafficGrid.  Center for Connected Learning and Computer-Based Modeling, Northwestern University, Evanston, IL.

In other publications, please use:  Copyright 2003 Uri Wilensky.  All rights reserved.  See http://ccl.northwestern.edu/netlogo/models/TrafficGrid for terms of use.
@#$#@#$#@
default
true
0
Polygon -7500403 true true 150 5 40 250 150 205 260 250

airplane
true
0
Polygon -7500403 true true 150 0 135 15 120 60 120 105 15 165 15 195 120 180 135 240 105 270 120 285 150 270 180 285 210 270 165 240 180 180 285 195 285 165 180 105 180 60 165 15

arrow
true
0
Polygon -7500403 true true 150 0 0 150 105 150 105 293 195 293 195 150 300 150

box
false
0
Polygon -7500403 true true 150 285 285 225 285 75 150 135
Polygon -7500403 true true 150 135 15 75 150 15 285 75
Polygon -7500403 true true 15 75 15 225 150 285 150 135
Line -16777216 false 150 285 150 135
Line -16777216 false 150 135 15 75
Line -16777216 false 150 135 285 75

bug
true
0
Circle -7500403 true true 96 182 108
Circle -7500403 true true 110 127 80
Circle -7500403 true true 110 75 80
Line -7500403 true 150 100 80 30
Line -7500403 true 150 100 220 30

bus
false
0
Polygon -7500403 true true 15 206 15 150 15 120 30 105 270 105 285 120 285 135 285 206 270 210 30 210
Rectangle -16777216 true false 36 126 231 159
Line -7500403 false 60 135 60 165
Line -7500403 false 60 120 60 165
Line -7500403 false 90 120 90 165
Line -7500403 false 120 120 120 165
Line -7500403 false 150 120 150 165
Line -7500403 false 180 120 180 165
Line -7500403 false 210 120 210 165
Line -7500403 false 240 135 240 165
Rectangle -16777216 true false 15 174 285 182
Circle -16777216 true false 48 187 42
Rectangle -16777216 true false 240 127 276 205
Circle -16777216 true false 195 187 42
Line -7500403 false 257 120 257 207

butterfly
true
0
Polygon -7500403 true true 150 165 209 199 225 225 225 255 195 270 165 255 150 240
Polygon -7500403 true true 150 165 89 198 75 225 75 255 105 270 135 255 150 240
Polygon -7500403 true true 139 148 100 105 55 90 25 90 10 105 10 135 25 180 40 195 85 194 139 163
Polygon -7500403 true true 162 150 200 105 245 90 275 90 290 105 290 135 275 180 260 195 215 195 162 165
Polygon -16777216 true false 150 255 135 225 120 150 135 120 150 105 165 120 180 150 165 225
Circle -16777216 true false 135 90 30
Line -16777216 false 150 105 195 60
Line -16777216 false 150 105 105 60

car
true
0
Polygon -7500403 true true 180 15 164 21 144 39 135 60 132 74 106 87 84 97 63 115 50 141 50 165 60 225 150 285 165 285 225 285 225 15 180 15
Circle -16777216 true false 180 30 90
Circle -16777216 true false 180 180 90
Polygon -16777216 true false 80 138 78 168 135 166 135 91 105 106 96 111 89 120
Circle -7500403 true true 195 195 58
Circle -7500403 true true 195 47 58

car side
false
0
Polygon -7500403 true true 19 147 11 125 16 105 63 105 99 79 155 79 180 105 243 111 266 129 253 149
Circle -16777216 true false 43 123 42
Circle -16777216 true false 194 124 42
Polygon -16777216 true false 101 87 73 108 171 108 151 87
Line -8630108 false 121 82 120 108
Polygon -1 true false 242 121 248 128 266 129 247 115
Rectangle -16777216 true false 12 131 28 143

car top
true
0
Polygon -7500403 true true 151 8 119 10 98 25 86 48 82 225 90 270 105 289 150 294 195 291 210 270 219 225 214 47 201 24 181 11
Polygon -16777216 true false 210 195 195 210 195 135 210 105
Polygon -16777216 true false 105 255 120 270 180 270 195 255 195 225 105 225
Polygon -16777216 true false 90 195 105 210 105 135 90 105
Polygon -1 true false 205 29 180 30 181 11
Line -7500403 false 210 165 195 165
Line -7500403 false 90 165 105 165
Polygon -16777216 true false 121 135 180 134 204 97 182 89 153 85 120 89 98 97
Line -16777216 false 210 90 195 30
Line -16777216 false 90 90 105 30
Polygon -1 true false 95 29 120 30 119 11

circle
false
0
Circle -7500403 true true 0 0 300

circle 2
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240

cow
false
0
Polygon -7500403 true true 200 193 197 249 179 249 177 196 166 187 140 189 93 191 78 179 72 211 49 209 48 181 37 149 25 120 25 89 45 72 103 84 179 75 198 76 252 64 272 81 293 103 285 121 255 121 242 118 224 167
Polygon -7500403 true true 73 210 86 251 62 249 48 208
Polygon -7500403 true true 25 114 16 195 9 204 23 213 25 200 39 123

cylinder
false
0
Circle -7500403 true true 0 0 300

dot
false
0
Circle -7500403 true true 90 90 120

face happy
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 255 90 239 62 213 47 191 67 179 90 203 109 218 150 225 192 218 210 203 227 181 251 194 236 217 212 240

face neutral
false
0
Circle -7500403 true true 8 7 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Rectangle -16777216 true false 60 195 240 225

face sad
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 168 90 184 62 210 47 232 67 244 90 220 109 205 150 198 192 205 210 220 227 242 251 229 236 206 212 183

fish
false
0
Polygon -1 true false 44 131 21 87 15 86 0 120 15 150 0 180 13 214 20 212 45 166
Polygon -1 true false 135 195 119 235 95 218 76 210 46 204 60 165
Polygon -1 true false 75 45 83 77 71 103 86 114 166 78 135 60
Polygon -7500403 true true 30 136 151 77 226 81 280 119 292 146 292 160 287 170 270 195 195 210 151 212 30 166
Circle -16777216 true false 215 106 30

flag
false
0
Rectangle -7500403 true true 60 15 75 300
Polygon -7500403 true true 90 150 270 90 90 30
Line -7500403 true 75 135 90 135
Line -7500403 true 75 45 90 45

flower
false
0
Polygon -10899396 true false 135 120 165 165 180 210 180 240 150 300 165 300 195 240 195 195 165 135
Circle -7500403 true true 85 132 38
Circle -7500403 true true 130 147 38
Circle -7500403 true true 192 85 38
Circle -7500403 true true 85 40 38
Circle -7500403 true true 177 40 38
Circle -7500403 true true 177 132 38
Circle -7500403 true true 70 85 38
Circle -7500403 true true 130 25 38
Circle -7500403 true true 96 51 108
Circle -16777216 true false 113 68 74
Polygon -10899396 true false 189 233 219 188 249 173 279 188 234 218
Polygon -10899396 true false 180 255 150 210 105 210 75 240 135 240

house
false
0
Rectangle -7500403 true true 45 120 255 285
Rectangle -16777216 true false 120 210 180 285
Polygon -7500403 true true 15 120 150 15 285 120
Line -16777216 false 30 120 270 120

leaf
false
0
Polygon -7500403 true true 150 210 135 195 120 210 60 210 30 195 60 180 60 165 15 135 30 120 15 105 40 104 45 90 60 90 90 105 105 120 120 120 105 60 120 60 135 30 150 15 165 30 180 60 195 60 180 120 195 120 210 105 240 90 255 90 263 104 285 105 270 120 285 135 240 165 240 180 270 195 240 210 180 210 165 195
Polygon -7500403 true true 135 195 135 240 120 255 105 255 105 285 135 285 165 240 165 195

line
true
0
Line -7500403 true 150 0 150 300

line half
true
0
Line -7500403 true 150 0 150 150

pentagon
false
0
Polygon -7500403 true true 150 15 15 120 60 285 240 285 285 120

person
false
0
Circle -7500403 true true 110 5 80
Polygon -7500403 true true 105 90 120 195 90 285 105 300 135 300 150 225 165 300 195 300 210 285 180 195 195 90
Rectangle -7500403 true true 127 79 172 94
Polygon -7500403 true true 195 90 240 150 225 180 165 105
Polygon -7500403 true true 105 90 60 150 75 180 135 105

plant
false
0
Rectangle -7500403 true true 135 90 165 300
Polygon -7500403 true true 135 255 90 210 45 195 75 255 135 285
Polygon -7500403 true true 165 255 210 210 255 195 225 255 165 285
Polygon -7500403 true true 135 180 90 135 45 120 75 180 135 210
Polygon -7500403 true true 165 180 165 210 225 180 255 120 210 135
Polygon -7500403 true true 135 105 90 60 45 45 75 105 135 135
Polygon -7500403 true true 165 105 165 135 225 105 255 45 210 60
Polygon -7500403 true true 135 90 120 45 150 15 180 45 165 90

square
false
0
Rectangle -7500403 true true 30 30 270 270

square 2
false
0
Rectangle -7500403 true true 30 30 270 270
Rectangle -16777216 true false 60 60 240 240

star
false
0
Polygon -7500403 true true 151 1 185 108 298 108 207 175 242 282 151 216 59 282 94 175 3 108 116 108

target
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240
Circle -7500403 true true 60 60 180
Circle -16777216 true false 90 90 120
Circle -7500403 true true 120 120 60

tree
false
0
Circle -7500403 true true 118 3 94
Rectangle -6459832 true false 120 195 180 300
Circle -7500403 true true 65 21 108
Circle -7500403 true true 116 41 127
Circle -7500403 true true 45 90 120
Circle -7500403 true true 104 74 152

triangle
false
0
Polygon -7500403 true true 150 30 15 255 285 255

triangle 2
false
0
Polygon -7500403 true true 150 30 15 255 285 255
Polygon -16777216 true false 151 99 225 223 75 224

truck
false
0
Rectangle -7500403 true true 4 45 195 187
Polygon -7500403 true true 296 193 296 150 259 134 244 104 208 104 207 194
Rectangle -1 true false 195 60 195 105
Polygon -16777216 true false 238 112 252 141 219 141 218 112
Circle -16777216 true false 234 174 42
Rectangle -7500403 true true 181 185 214 194
Circle -16777216 true false 144 174 42
Circle -16777216 true false 24 174 42
Circle -7500403 false true 24 174 42
Circle -7500403 false true 144 174 42
Circle -7500403 false true 234 174 42

turtle
true
0
Polygon -10899396 true false 215 204 240 233 246 254 228 266 215 252 193 210
Polygon -10899396 true false 195 90 225 75 245 75 260 89 269 108 261 124 240 105 225 105 210 105
Polygon -10899396 true false 105 90 75 75 55 75 40 89 31 108 39 124 60 105 75 105 90 105
Polygon -10899396 true false 132 85 134 64 107 51 108 17 150 2 192 18 192 52 169 65 172 87
Polygon -10899396 true false 85 204 60 233 54 254 72 266 85 252 107 210
Polygon -7500403 true true 119 75 179 75 209 101 224 135 220 225 175 261 128 261 81 224 74 135 88 99

van side
false
0
Polygon -7500403 true true 26 147 18 125 36 61 161 61 177 67 195 90 242 97 262 110 273 129 260 149
Circle -16777216 true false 43 123 42
Circle -16777216 true false 194 124 42
Polygon -16777216 true false 45 68 37 95 183 96 169 69
Line -7500403 true 62 65 62 103
Line -7500403 true 115 68 120 100
Polygon -1 true false 271 127 258 126 257 114 261 109
Rectangle -16777216 true false 19 131 27 142

van top
true
0
Polygon -7500403 true true 90 117 71 134 228 133 210 117
Polygon -7500403 true true 150 8 118 10 96 17 85 30 84 264 89 282 105 293 149 294 192 293 209 282 215 265 214 31 201 17 179 10
Polygon -16777216 true false 94 129 105 120 195 120 204 128 180 150 120 150
Polygon -16777216 true false 90 270 105 255 105 150 90 135
Polygon -16777216 true false 101 279 120 286 180 286 198 281 195 270 105 270
Polygon -16777216 true false 210 270 195 255 195 150 210 135
Polygon -1 true false 201 16 201 26 179 20 179 10
Polygon -1 true false 99 16 99 26 121 20 121 10
Line -16777216 false 130 14 168 14
Line -16777216 false 130 18 168 18
Line -16777216 false 130 11 168 11
Line -16777216 false 185 29 194 112
Line -16777216 false 115 29 106 112
Line -7500403 false 210 180 195 180
Line -7500403 false 195 225 210 240
Line -7500403 false 105 225 90 240
Line -7500403 false 90 180 105 180

wheel
false
0
Circle -7500403 true true 3 3 294
Circle -16777216 true false 30 30 240
Line -7500403 true 150 285 150 15
Line -7500403 true 15 150 285 150
Circle -7500403 true true 120 120 60
Line -7500403 true 216 40 79 269
Line -7500403 true 40 84 269 221
Line -7500403 true 40 216 269 79
Line -7500403 true 84 40 221 269

x
false
0
Polygon -7500403 true true 270 75 225 30 30 225 75 270
Polygon -7500403 true true 30 75 75 30 270 225 225 270

@#$#@#$#@
NetLogo 5.0.4
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
<experiments>
  <experiment name="default" repetitions="1" runMetricsEveryStep="false">
    <setup>setup</setup>
    <go>go</go>
    <enumeratedValueSet variable="random-seed">
      <value value="111111"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="tick_limit">
      <value value="3000"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ticks_to_alarm">
      <value value="838"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="num_cars">
      <value value="1500"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="evac_file">
      <value value="&quot;evac_00123&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="out_file">
      <value value="&quot;00123&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="inter_file">
      <value value="&quot;inter_simple.txt&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="percent_fast">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="percent_slow">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="percent_leaving">
      <value value="50"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="emergency?">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="speed_limit">
      <value value="1"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="experiment" repetitions="1" runMetricsEveryStep="false">
    <setup>setup</setup>
    <go>go
if (ticks mod 600 = 0)
[
export-view (word ticks " ticks.png")
show-gas
export-view (word ticks " plume.png")
reset-patch-colors
]</go>
    <enumeratedValueSet variable="emergency?">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="#is_gui?">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="TICKS_TO_ALARM">
      <value value="150"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="EXTRA_RESPAWN">
      <value value="0"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="*who">
      <value value="2825"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="plume?">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="seed_decimal">
      <value value="0.1192019331"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="garage?">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NUM_CARS">
      <value value="5000"/>
    </enumeratedValueSet>
  </experiment>
</experiments>
@#$#@#$#@
@#$#@#$#@
default
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180

@#$#@#$#@
1
@#$#@#$#@
