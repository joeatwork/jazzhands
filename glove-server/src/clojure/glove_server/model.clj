(ns glove-server.model)

;; atan2(2*qy*qw-2*qx*qz , 1 - 2*qy2 - 2*qz2)

(defn quaternion->euler [[q1 q2 q3 q4]]
  {:heading (Math/atan2 (- (* 2 q2 q3) (* 2 q1 q4)) (+ (* 2 q1 q1) (* 2 q2 q2) -1))
   :attitude (Math/asin (+ (* 2 q2 q4) (* 2 q1 q3)))
   :bank (Math/atan2 (- (* 2 q3 q4) (* 2 q2 q2)) (+ (* 2 q1 q1) (* 2 q4 q4) -1))})
