;; Auto-generated. Do not edit!


(when (boundp 'taskgen::Order)
  (if (not (find-package "TASKGEN"))
    (make-package "TASKGEN"))
  (shadow 'Order (find-package "TASKGEN")))
(unless (find-package "TASKGEN::ORDER")
  (make-package "TASKGEN::ORDER"))

(in-package "ROS")
;;//! \htmlinclude Order.msg.html


(defclass taskgen::Order
  :super ros::object
  :slots (_deadline _x_position _y_position _z_orientation ))

(defmethod taskgen::Order
  (:init
   (&key
    ((:deadline __deadline) 0.0)
    ((:x_position __x_position) 0.0)
    ((:y_position __y_position) 0.0)
    ((:z_orientation __z_orientation) 0.0)
    )
   (send-super :init)
   (setq _deadline (float __deadline))
   (setq _x_position (float __x_position))
   (setq _y_position (float __y_position))
   (setq _z_orientation (float __z_orientation))
   self)
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

(setf (get taskgen::Order :md5sum-) "260c63c5cf6dcaef40226fcd5deee22b")
(setf (get taskgen::Order :datatype-) "taskgen/Order")
(setf (get taskgen::Order :definition-)
      "float64 deadline
float64 x_position
float64 y_position
float64 z_orientation

")



(provide :taskgen/Order "260c63c5cf6dcaef40226fcd5deee22b")


