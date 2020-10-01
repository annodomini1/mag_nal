;; Auto-generated. Do not edit!


(when (boundp 'taskgen3::Task)
  (if (not (find-package "TASKGEN3"))
    (make-package "TASKGEN3"))
  (shadow 'Task (find-package "TASKGEN3")))
(unless (find-package "TASKGEN3::TASK")
  (make-package "TASKGEN3::TASK"))

(in-package "ROS")
;;//! \htmlinclude Task.msg.html


(defclass taskgen3::Task
  :super ros::object
  :slots (_task_id _deadline _x_position _y_position _z_orientation ))

(defmethod taskgen3::Task
  (:init
   (&key
    ((:task_id __task_id) 0)
    ((:deadline __deadline) 0.0)
    ((:x_position __x_position) 0.0)
    ((:y_position __y_position) 0.0)
    ((:z_orientation __z_orientation) 0.0)
    )
   (send-super :init)
   (setq _task_id (round __task_id))
   (setq _deadline (float __deadline))
   (setq _x_position (float __x_position))
   (setq _y_position (float __y_position))
   (setq _z_orientation (float __z_orientation))
   self)
  (:task_id
   (&optional __task_id)
   (if __task_id (setq _task_id __task_id)) _task_id)
  (:deadline
   (&optional __deadline)
   (if __deadline (setq _deadline __deadline)) _deadline)
  (:x_position
   (&optional __x_position)
   (if __x_position (setq _x_position __x_position)) _x_position)
  (:y_position
   (&optional __y_position)
   (if __y_position (setq _y_position __y_position)) _y_position)
  (:z_orientation
   (&optional __z_orientation)
   (if __z_orientation (setq _z_orientation __z_orientation)) _z_orientation)
  (:serialization-length
   ()
   (+
    ;; uint16 _task_id
    2
    ;; float64 _deadline
    8
    ;; float64 _x_position
    8
    ;; float64 _y_position
    8
    ;; float64 _z_orientation
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _task_id
       (write-word _task_id s)
     ;; float64 _deadline
       (sys::poke _deadline (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x_position
       (sys::poke _x_position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_position
       (sys::poke _y_position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z_orientation
       (sys::poke _z_orientation (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _task_id
     (setq _task_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _deadline
     (setq _deadline (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x_position
     (setq _x_position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_position
     (setq _y_position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z_orientation
     (setq _z_orientation (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get taskgen3::Task :md5sum-) "61469da423d711030840b17fc9caf72f")
(setf (get taskgen3::Task :datatype-) "taskgen3/Task")
(setf (get taskgen3::Task :definition-)
      "uint16 task_id
float64 deadline
float64 x_position
float64 y_position
float64 z_orientation

")



(provide :taskgen3/Task "61469da423d711030840b17fc9caf72f")


