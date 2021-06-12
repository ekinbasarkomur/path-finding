;; Auto-generated. Do not edit!


(when (boundp 'pathfinding::Target)
  (if (not (find-package "PATHFINDING"))
    (make-package "PATHFINDING"))
  (shadow 'Target (find-package "PATHFINDING")))
(unless (find-package "PATHFINDING::TARGET")
  (make-package "PATHFINDING::TARGET"))

(in-package "ROS")
;;//! \htmlinclude Target.msg.html


(defclass pathfinding::Target
  :super ros::object
  :slots (_x _y ))

(defmethod pathfinding::Target
  (:init
   (&key
    ((:x __x) 0)
    ((:y __y) 0)
    )
   (send-super :init)
   (setq _x (round __x))
   (setq _y (round __y))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; uint8 _x
    1
    ;; uint8 _y
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _x
       (write-byte _x s)
     ;; uint8 _y
       (write-byte _y s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _x
     (setq _x (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _y
     (setq _y (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get pathfinding::Target :md5sum-) "727012f6868afa655d78dc8b436d2c91")
(setf (get pathfinding::Target :datatype-) "pathfinding/Target")
(setf (get pathfinding::Target :definition-)
      "uint8 x
uint8 y

")



(provide :pathfinding/Target "727012f6868afa655d78dc8b436d2c91")


