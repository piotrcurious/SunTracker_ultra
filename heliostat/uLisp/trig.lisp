;; Configuration Constants
(defconstant xOffset 1.0)   ;; X Offset in meters
(defconstant yOffset 2.0)   ;; Y Offset in meters
(defconstant zOffset 0.5)   ;; Z Offset in meters
(defconstant targetX 0.0)   ;; Target X in meters
(defconstant targetY 0.0)   ;; Target Y in meters
(defconstant targetZ 1.0)   ;; Target Z in meters
(defconstant steps-per-degree 10)  ;; Steps per degree
(defconstant max-speed 200.0)      ;; Max speed in steps/sec
(defconstant acceleration 100.0)   ;; Acceleration in steps/sec^2

;; Hardware Pin Definitions
(defconstant base-step-pin 4)
(defconstant base-dir-pin 5)
(defconstant tilt-step-pin 6)
(defconstant tilt-dir-pin 7)
(defconstant base-end-stop-pin 2)
(defconstant tilt-end-stop-pin 3)

;; Mathematical Operations
(defun normalize-vector (x y z)
  (let* ((magnitude (sqrt (+ (* x x) (* y y) (* z z)))))
    (list (/ x magnitude) (/ y magnitude) (/ z magnitude))))

(defun calculate-base-angle (x y)
  (* (/ (atan2 y x) pi) 180))

(defun calculate-tilt-angle (z)
  (* (/ (asin z) pi) 180))

;; Stepper Motor Control
(defun move-stepper (step-pin dir-pin steps speed)
  (let ((direction (if (> steps 0) 1 0))
        (steps (abs steps)))
    (digitalwrite dir-pin direction)
    (dotimes (i steps)
      (digitalwrite step-pin 1)
      (delay (/ 1000 speed))
      (digitalwrite step-pin 0)
      (delay (/ 1000 speed)))))

(defun reset-stepper (step-pin dir-pin end-stop-pin)
  (digitalwrite dir-pin 0)
  (loop
   (when (not (digitalread end-stop-pin)) (return))
   (digitalwrite step-pin 1)
   (delay 5)
   (digitalwrite step-pin 0)
   (delay 5))
  0)

;; Heliostat Logic
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

(defun heliostat-update (sun-azimuth sun-elevation)
  (let* ((heliostat-pos (list xOffset yOffset zOffset))
         (target-pos (list targetX targetY targetZ))
         (target-dir (normalize-vector
                      (- (nth 0 target-pos) (nth 0 heliostat-pos))
                      (- (nth 1 target-pos) (nth 1 heliostat-pos))
                      (- (nth 2 target-pos) (nth 2 heliostat-pos))))
         (sun-dir (normalize-vector
                   (cos (radians sun-azimuth))
                   (sin (radians sun-azimuth))
                   (sin (radians sun-elevation))))
         (mirror-normal (normalize-vector
                         (+ (nth 0 sun-dir) (nth 0 target-dir))
                         (+ (nth 1 sun-dir) (nth 1 target-dir))
                         (+ (nth 2 sun-dir) (nth 2 target-dir))))
         (base-angle (calculate-base-angle (nth 0 mirror-normal) (nth 1 mirror-normal)))
         (tilt-angle (calculate-tilt-angle (nth 2 mirror-normal)))
         (base-steps (* steps-per-degree base-angle))
         (tilt-steps (* steps-per-degree tilt-angle)))
    (progn
      (move-stepper base-step-pin base-dir-pin base-steps max-speed)
      (move-stepper tilt-step-pin tilt-dir-pin tilt-steps max-speed))))

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
