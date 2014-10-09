(ns glove-server.instruments
  (:use [overtone.live]))

(definst triangle-wave [freq 440 sustain 0.4 vol 0.4]
  (* (env-gen (triangle) 1 1 0 1 FREE)
     (lf-tri freq)
     vol))
