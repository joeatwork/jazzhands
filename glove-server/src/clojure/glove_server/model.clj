(ns glove-server.model)

;; atan2(2*qy*qw-2*qx*qz , 1 - 2*qy2 - 2*qz2)

(defn quaternion->euler [[q1 q2 q3 q4]]
  {:heading (Math/atan2 (- (* 2 q2 q3) (* 2 q1 q4)) (+ (* 2 q1 q1) (* 2 q2 q2) -1))
   :attitude (Math/asin (+ (* 2 q2 q4) (* 2 q1 q3)))
   :bank (Math/atan2 (- (* 2 q3 q4) (* 2 q2 q2)) (+ (* 2 q1 q1) (* 2 q4 q4) -1))})

(defn accel->mag->euler [[accel-x accel-y accel-z] [mag-x mag-y mag-z]]
  (let [bank (Math/atan2 accel-y accel-z) ;; phi
        attitude (Math/atan2 (- accel-x) (Math/sqrt (+ (* accel-y accel-y) (* accel-z accel-z)))) ;; theta
        sin-bank (Math/sin bank)
        cos-bank (Math/cos bank)
        sin-attitude (Math/sin attitude)
        heading (Math/atan2 (- (* mag-z sin-bank) (* mag-y cos-bank))
                            (+ (* mag-x (Math/cos attitude))
                               (* mag-y sin-attitude sin-bank)
                               (* mag-z sin-attitude cos-bank)))]
    {:bank bank :attitude attitude :heading heading}))
