;; Configuration Constants
(defconstant xOffset 1.0)  ;; Heliostat base X-offset
(defconstant yOffset 2.0)  ;; Heliostat base Y-offset
(defconstant zOffset 0.5)  ;; Heliostat base Z-offset
(defconstant targetX 0.0)  ;; Target X-coordinate
(defconstant targetY 0.0)  ;; Target Y-coordinate
(defconstant targetZ 1.0)  ;; Target Z-coordinate
(defconstant steps-per-degree 10)  ;; Steps per degree for motors
(defconstant max-speed 200.0)      ;; Maximum motor speed
(defconstant acceleration 100.0)   ;; Motor acceleration rate

;; Hardware Pin Definitions
(defconstant base-step-pin 4)
(defconstant base-dir-pin 5)
(defconstant tilt-step-pin 6)
(defconstant tilt-dir-pin 7)
(defconstant base-end-stop-pin 2)
(defconstant tilt-end-stop-pin 3)

;; Field-Based Vector Operations
(defun vector-add (a b)
  (mapcar #'+ a b))

(defun vector-subtract (a b)
  (mapcar #'- a b))

(defun vector-scale (v scalar)
  (mapcar (lambda (x) (* x scalar)) v))

(defun vector-dot (a b)
  (reduce #'+ (mapcar #'* a b)))

(defun vector-cross (a b)
  (list (- (* (nth 1 a) (nth 2 b)) (* (nth 2 a) (nth 1 b)))
        (- (* (nth 2 a) (nth 0 b)) (* (nth 0 a) (nth 2 b)))
        (- (* (nth 0 a) (nth 1 b)) (* (nth 1 a) (nth 0 b)))))

(defun vector-magnitude-squared (v)
  (vector-dot v v))

(defun vector-normalize (v)
  (let ((mag (sqrt (vector-magnitude-squared v))))
    (if (> mag 0) (vector-scale v (/ 1.0 mag)) v)))

;; Homogeneous Coordinates Transformation
(defun to-homogeneous (v)
  (append v '(1.0)))

(defun from-homogeneous (v)
  (let ((w (nth 3 v)))
    (if (> (abs w) 1e-9)
        (mapcar (lambda (x) (/ x w)) (subseq v 0 3))
        (subseq v 0 3))))

;; Angle Calculations
(defun angle-dot-product (v1 v2)
  (let ((dot (vector-dot v1 v2))
        (mag1 (sqrt (vector-magnitude-squared v1)))
        (mag2 (sqrt (vector-magnitude-squared v2))))
    (if (and (> mag1 0) (> mag2 0))
        (/ dot (* mag1 mag2))
        0)))

;; Mirror Normal Calculation
(defun calculate-mirror-normal (sun-dir target-dir)
  (vector-normalize (vector-add sun-dir target-dir)))

;; Smoothed Stepper Motor Control
(defun move-stepper (step-pin dir-pin steps speed accel)
  (let ((direction (if (> steps 0) 1 0))
        (steps (abs steps))
        (current-speed 0.0)
        (step-delay (/ 1000 speed)))
    (digitalwrite dir-pin direction)
    (dotimes (i steps)
      ;; Ramp up or down speed
      (setq current-speed
            (min speed (+ current-speed (/ accel steps))))
      (setq step-delay (/ 1000 current-speed))
      ;; Step
      (digitalwrite step-pin 1)
      (delay step-delay)
      (digitalwrite step-pin 0)
      (delay step-delay))))

(defun reset-stepper (step-pin dir-pin end-stop-pin)
  (digitalwrite dir-pin 0)
  (loop
   (when (not (digitalread end-stop-pin)) (return))
   (digitalwrite step-pin 1)
   (delay 5)
   (digitalwrite step-pin 0)
   (delay 5))
  0)

;; Heliostat Initialization
(defun heliostat-initialize ()
  (progn
    (pinmode base-step-pin t)
    (pinmode base-dir-pin t)
    (pinmode tilt-step-pin t)
    (pinmode tilt-dir-pin t)
    (pinmode base-end-stop-pin nil)
    (pinmode tilt-end-stop-pin nil)
    (reset-stepper base-step-pin base-dir-pin base-end-stop-pin)
    (reset-stepper tilt-step-pin tilt-dir-pin tilt-end-stop-pin)))

;; Heliostat Control
(defun heliostat-update (sun-azimuth sun-elevation)
  (let* ((heliostat-pos (list xOffset yOffset zOffset))
         (target-pos (list targetX targetY targetZ))
         (target-dir (vector-normalize (vector-subtract target-pos heliostat-pos)))
         (sun-dir (vector-normalize
                   (list (cos (radians sun-azimuth))
                         (sin (radians sun-azimuth))
                         (sin (radians sun-elevation)))))
         (mirror-normal (calculate-mirror-normal sun-dir target-dir))
         (base-angle (angle-dot-product mirror-normal (list 1.0 0.0 0.0)))
         (tilt-angle (angle-dot-product mirror-normal (list 0.0 0.0 1.0)))
         (base-steps (round (* steps-per-degree base-angle)))
         (tilt-steps (round (* steps-per-degree tilt-angle))))
    (progn
      (move-stepper base-step-pin base-dir-pin base-steps max-speed acceleration)
      (move-stepper tilt-step-pin tilt-dir-pin tilt-steps max-speed acceleration))))

;; Main Program
(defun setup ()
  (progn
    (heliostat-initialize)
    (print "Heliostat Initialized.")))

(defun loop ()
  (let ((sun-azimuth 45)   ;; Example sun azimuth in degrees
        (sun-elevation 30)) ;; Example sun elevation in degrees
    (heliostat-update sun-azimuth sun-elevation)
    (delay 1000))) ;; Update every second
