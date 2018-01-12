;; Auto-generated. Do not edit!


(when (boundp 'auto_flight::destinate)
  (if (not (find-package "AUTO_FLIGHT"))
    (make-package "AUTO_FLIGHT"))
  (shadow 'destinate (find-package "AUTO_FLIGHT")))
(unless (find-package "AUTO_FLIGHT::DESTINATE")
  (make-package "AUTO_FLIGHT::DESTINATE"))
(unless (find-package "AUTO_FLIGHT::DESTINATEREQUEST")
  (make-package "AUTO_FLIGHT::DESTINATEREQUEST"))
(unless (find-package "AUTO_FLIGHT::DESTINATERESPONSE")
  (make-package "AUTO_FLIGHT::DESTINATERESPONSE"))

(in-package "ROS")





(defclass auto_flight::destinateRequest
  :super ros::object
  :slots (_x _y _z ))

(defmethod auto_flight::destinateRequest
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass auto_flight::destinateResponse
  :super ros::object
  :slots (_des_r ))

(defmethod auto_flight::destinateResponse
  (:init
   (&key
    ((:des_r __des_r) nil)
    )
   (send-super :init)
   (setq _des_r __des_r)
   self)
  (:des_r
   (&optional __des_r)
   (if __des_r (setq _des_r __des_r)) _des_r)
  (:serialization-length
   ()
   (+
    ;; bool _des_r
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _des_r
       (if _des_r (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _des_r
     (setq _des_r (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass auto_flight::destinate
  :super ros::object
  :slots ())

(setf (get auto_flight::destinate :md5sum-) "685a8a8431827e326400ebccd41b7c28")
(setf (get auto_flight::destinate :datatype-) "auto_flight/destinate")
(setf (get auto_flight::destinate :request) auto_flight::destinateRequest)
(setf (get auto_flight::destinate :response) auto_flight::destinateResponse)

(defmethod auto_flight::destinateRequest
  (:response () (instance auto_flight::destinateResponse :init)))

(setf (get auto_flight::destinateRequest :md5sum-) "685a8a8431827e326400ebccd41b7c28")
(setf (get auto_flight::destinateRequest :datatype-) "auto_flight/destinateRequest")
(setf (get auto_flight::destinateRequest :definition-)
      "float64 x
float64 y
float64 z
---
bool des_r
")

(setf (get auto_flight::destinateResponse :md5sum-) "685a8a8431827e326400ebccd41b7c28")
(setf (get auto_flight::destinateResponse :datatype-) "auto_flight/destinateResponse")
(setf (get auto_flight::destinateResponse :definition-)
      "float64 x
float64 y
float64 z
---
bool des_r
")



(provide :auto_flight/destinate "685a8a8431827e326400ebccd41b7c28")


