(require :robot-interface "package://pr2eus/robot-interface.l")

(ros::load-ros-manifest "control_msgs")
(ros::load-ros-manifest "kxr_controller")


(defun load-robot-model (&key (port 8000) (namespace nil)
                           (server-ip "localhost"))
  (let* ((hash-param-name (if namespace (format nil "~A~A" namespace "/eusmodel_hash") "/eusmodel_hash"))
         (robot-name-param-name (if namespace (format nil "~A~A" namespace "/eus_robot_name") "/eus_robot_name"))
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

    (when (not (probe-file outpath))
      (while (not (= (unix:system (format nil "wget -O ~A ~A" outpath server-url)) 0))
        (ros::ros-info (format nil "Waiting model file from server url ~A" server-url))
        (unix::usleep (* 1000 1000))))
    (load outpath)
    (funcall (read-from-string robot-name))))


(defclass kxr-interface
  :super robot-interface
  :slots (joint-names servo-on-off-client controller-name))


(defmethod kxr-interface
  (:init (robot &rest args &key (type :default-controller) &allow-other-keys)
   (let* ((namespace (cadr (memq :namespace args)))
          (c-name (cadr (memq :controller-name args)))
          (joint-param (if namespace (format nil "~A/~A/joints" namespace c-name)
                           (format nil "/~A/joints" c-name))))
     (setq controller-name (cadr (memq :controller-name args)))
     (setq joint-names (ros::get-param joint-param nil))
     (while (and (ros::ok) (null joint-names))
       (ros::ros-info (format nil "Waiting ~A rosparam set" joint-param))
       (setq joint-names (ros::get-param joint-param nil))
       (unix::usleep (* 1000 1000))))

   (send-super* :init :robot robot :type type
                :groupname "kxr_interface"
                args)

   (setq servo-on-off-client (instance ros::simple-action-client :init
                                       (if namespace (format nil "/~A/~A/servo_on_off" namespace controller-name)
                                           (format nil "/~A/servo_on_off" controller-name))
                                       kxr_controller::ServoOnOffAction
                                       :groupname groupname))
   (setq stretch-client (instance ros::simple-action-client :init
                                  (if namespace (format nil "/~A/~A/stretch_interface" namespace controller-name)
                                    (format nil "/~A/stretch_interface" controller-name))
                                  kxr_controller::StretchAction
                                  :groupname groupname))
   (setq pressure-control-client (instance ros::simple-action-client :init
                                  (if namespace (format nil "/~A/~A/pressure_control_interface" namespace controller-name)
                                    (format nil "/~A/pressure_control_interface" controller-name))
                                  kxr_controller::PressureControlAction
                                  :groupname groupname))
   (dolist (action (list servo-on-off-client stretch-client pressure-control-client))
     (unless (and joint-action-enable (send action :wait-for-server 3))
       (setq joint-action-enable nil)
       (ros::ros-warn "~A is not respond, kxr-interface is disabled" action)
       (return)))
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
   (unless joint-action-enable
     (if viewer (send self :draw-objects))
     (return-from :servo-on t))
   (when (null names)
     (setq names joint-names))
   (let* (goal result)
     ;; send current angle-vector
     (while (ros::ok)
       (send self :angle-vector (send self :state :potentio-vector) 1.0)
       (send self :wait-interpolation)
       (setq tmp-result (send-all controller-actions :get-result))
       (when (every #'identity (send-all controller-actions :get-result))
         (return))
       (ros::ros-warn "waiting for the angle vector to be sent."))

     (setq goal (instance kxr_controller::ServoOnOffGoal :init))
     (send goal :joint_names names)
     (send goal :servo_on_states (make-array (length names) :initial-element t))
     (send servo-on-off-client :send-goal goal)))
  (:servo-off
   (&key (names nil))
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
   (when (null names)
     (setq names joint-names))
   (let* (goal result)
     (setq goal (instance kxr_controller::StretchGoal :init))
     (send goal :joint_names names)
     (send goal :stretch value)
     (send stretch-client :send-goal goal)))
  (:read-stretch
   ()
   (one-shot-subscribe
    (if namespace (format nil "/~A/~A/stretch" namespace controller-name)
      (format nil "/~A/stretch" controller-name))
    kxr_controller::Stretch))
  (:send-pressure-control
   (&key board-idx threshold release)
   (let* (goal result)
     (setq goal (instance kxr_controller::PressureControlGoal :init))
     (send goal :board_idx board-idx)
     (send goal :threshold threshold)
     (send goal :release release)
     (send pressure-control-client :send-goal goal)))
  (:read-pressure
   (&key board-idx)
   (one-shot-subscribe
    (if namespace (format nil "/~A/~A/pressure/~A" namespace controller-name board-idx)
      (format nil "/~A/pressure/~A" controller-name board-idx))
    std_msgs::Float32)))


(defun kxr-init (&key
                   (namespace nil)
                   (create-viewer t)
                   (controller-name "kxr_fullbody_controller"))
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
