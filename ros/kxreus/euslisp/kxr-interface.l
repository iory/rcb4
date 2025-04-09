(require :robot-interface "package://pr2eus/robot-interface.l")

(ros::load-ros-manifest "control_msgs")
(ros::load-ros-manifest "kxr_controller")


(defun load-robot-model (&key (port 8000) (namespace nil)
                           (server-ip "localhost"))
  (let* ((hash-param-name (if namespace (format nil "~A~A" namespace "/eusmodel_hash") "/eusmodel_hash"))
         (robot-name-param-name (if namespace (format nil "~A~A" namespace "/eus_robot_name") "/eus_robot_name"))
         (overwrite nil)
         (exit-flag nil)
         model
         hash robot-name outpath server-url)
    (setq port (truncate port))
    (while (null hash)
      (ros::ros-info (format nil "Waiting rosparam ~A" hash-param-name))
      (setq hash (ros::get-param hash-param-name nil))
      (unix::usleep (* 1000 1000)))
    (ros::ros-info (format nil "Get rosparam ~A" hash-param-name))

    (while (null robot-name)
      (ros::ros-info (format nil "Waiting rosparam ~A" robot-name-param-name))
      (setq robot-name (ros::get-param robot-name-param-name nil))
      (unix::usleep (* 1000 1000)))
    (ros::ros-info (format nil "Get rosparam ~A" robot-name-param-name))

    (setq outpath (ros::resolve-ros-path (format nil "package://kxreus/models/cache/~A.l" hash)))
    (setq server-url (format nil "http://~A:~A/euslisp/~A.l" server-ip port hash))

    (while (null exit-flag)
      (when (or (not (probe-file outpath)) overwrite)
        (ros::ros-info (format nil "Download model file from server url ~A" server-url))
        (while (not (= (unix:system (format nil "wget -O ~A ~A" outpath server-url)) 0))
          (ros::ros-info (format nil "Waiting model file from server url ~A" server-url))
          (unix::usleep (* 1000 1000))))
      (load outpath)
      (when (fboundp (read-from-string robot-name))
        (setq model (funcall (read-from-string robot-name)))
        (setq exit-flag t))
      (setq overwrite t))
    model))


(defclass kxr-interface
  ;; :super robot-interface
  :super robot-move-base-interface
  :slots (joint-names servo-on-off-client controller-name control-pressure use-sim-time))


(defmethod kxr-interface
  (:init (robot &rest args &key (type :default-controller)
                (input-joint-names nil) &allow-other-keys)
   (let* ((namespace (cadr (memq :namespace args)))
          (c-name (cadr (memq :controller-name args)))
          (joint-param (if namespace (format nil "~A/~A/joints" namespace c-name)
                         (format nil "/~A/joints" c-name)))
          (pressure-param (if namespace (format nil "~A/rcb4_ros_bridge/control_pressure" namespace)
                            (format nil "/rcb4_ros_bridge/control_pressure"))))
     (setq control-pressure (ros::get-param pressure-param nil))
     (setq controller-name (cadr (memq :controller-name args)))
     (setq joint-names (ros::get-param joint-param input-joint-names))
     (while (and (ros::ok) (null joint-names))
       (ros::ros-info (format nil "Waiting ~A rosparam set" joint-param))
       (setq joint-names (ros::get-param joint-param nil))
       (unix::usleep (* 1000 1000))))

   (send-super* :init :robot robot :type type
                :groupname "kxr_interface"
                :base-controller-action-name nil
                :base-frame-id "base_link"
                :cmd-vel-topic "/ridgeback_control/cmd_vel"
                args)

    (let (actions)
      (setq use-sim-time (and (ros::has-param "/use_sim_time") (ros::get-param "/use_sim_time")))
      (when (not use-sim-time)
        (setq servo-on-off-client (instance ros::simple-action-client :init
                                            (if namespace (format nil "/~A/~A/servo_on_off_real_interface" namespace controller-name)
                                                (format nil "/~A/servo_on_off_real_interface" controller-name))
                                            kxr_controller::ServoOnOffAction
                                            :groupname groupname))
        (setq stretch-client (instance ros::simple-action-client :init
                                       (if namespace (format nil "/~A/~A/stretch_interface" namespace controller-name)
                                           (format nil "/~A/stretch_interface" controller-name))
                                       kxr_controller::StretchAction
                                       :groupname groupname))
        (setq actions (list servo-on-off-client stretch-client))
        (when control-pressure
          (setq pressure-control-client (instance ros::simple-action-client :init
                                                  (if namespace (format nil "/~A/~A/pressure_control_interface" namespace controller-name)
                                                      (format nil "/~A/pressure_control_interface" controller-name))
                                                  kxr_controller::PressureControlAction
                                                  :groupname groupname))
          (setq actions (list servo-on-off-client stretch-client pressure-control-client))))
      (dolist (action actions)
        (unless (and joint-action-enable (send action :wait-for-server 3))
          (setq joint-action-enable nil)
          (ros::ros-warn "~A is not respond, kxr-interface is disabled" action)
          (return))))
   t)
  (:default-controller
   ()
   (send self :fullbody-controller))
  (:fullbody-controller
   ()
   (list
    (list
     (cons :controller-action (format nil"~A/follow_joint_trajectory" controller-name))
     (cons :controller-state (format nil "~A/state" controller-name))
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names joint-names))))
  (:servo-on
   (&key (names nil))
   (when use-sim-time
     (return-from :servo-on t))
   (unless joint-action-enable
     (if viewer (send self :draw-objects))
     (return-from :servo-on t))
   (when (null names)
     (setq names joint-names))
   (let* (goal result)
     (setq goal (instance kxr_controller::ServoOnOffGoal :init))
     (send goal :joint_names names)
     (send goal :servo_on_states (make-array (length names) :initial-element t))
     (send servo-on-off-client :send-goal goal)))
  (:servo-off
   (&key (names nil))
   (when use-sim-time
     (return-from :servo-on t))
   (unless joint-action-enable
     (if viewer (send self :draw-objects))
     (return-from :servo-off t))
   (when (null names)
     (setq names joint-names))
   (let* (goal result)
     (setq goal (instance kxr_controller::ServoOnOffGoal :init))
     (send goal :joint_names names)
     (send goal :servo_on_states (make-array (length names) :initial-element nil))
     (send servo-on-off-client :send-goal goal)))
  (:send-stretch
   (&key (value 127) (names nil))
   (when use-sim-time
     (return-from :servo-on t))
   (when (null names)
     (setq names joint-names))
   (let* (goal result)
     (setq goal (instance kxr_controller::StretchGoal :init))
     (send goal :joint_names names)
     (send goal :stretch value)
     (send stretch-client :send-goal goal)))
  (:read-stretch
   ()
   (when use-sim-time
     (return-from :servo-on t))
   (one-shot-subscribe
    (if namespace (format nil "/~A/~A/stretch" namespace controller-name)
      (format nil "/~A/stretch" controller-name))
    kxr_controller::Stretch))
  (:send-pressure-control
   (&key board-idx start-pressure stop-pressure release)
   (when use-sim-time
     (return-from :servo-on t))
   (let* (goal result)
     (setq goal (instance kxr_controller::PressureControlGoal :init))
     (send goal :board_idx board-idx)
     (send goal :start_pressure start-pressure)
     (send goal :stop_pressure stop-pressure)
     (send goal :release release)
     (send pressure-control-client :send-goal goal)))
  (:read-pressure
   (&key board-idx)
   (when use-sim-time
     (return-from :servo-on t))
   (one-shot-subscribe
    (if namespace (format nil "/~A/~A/pressure/~A" namespace controller-name board-idx)
      (format nil "/~A/pressure/~A" controller-name board-idx))
    std_msgs::Float32)))


(defun kxr-init (&key
                   (namespace nil)
                   (create-viewer t)
                   (controller-name "fullbody_controller"))
  (unless (boundp '*robot*)
    (setq *robot* (load-robot-model :namespace namespace
                                    :server-ip (ros::get-param (if namespace (format nil "~A/model_server_ip" namespace) "/model_server_ip") "localhost")
                                    :port (ros::get-param (if namespace (format nil "~A/model_server_port" namespace) "/model_server_port") 8123))))
  (unless (ros::ok) (ros::roseus "kxr_eus_interface"))
  (unless (boundp '*ri*)
    (setq *ri* (instance kxr-interface :init *robot* :namespace namespace
                         :controller-name controller-name)))

  (ros::spin-once)
  (send *ri* :spin-once)
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (when create-viewer (objects (list *robot*))))
