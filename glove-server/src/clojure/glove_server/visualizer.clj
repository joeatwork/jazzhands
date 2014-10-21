(ns glove-server.visualizer
  (:require [quil.core :as quil]
            [glove-server.serial :as serial]
            [glove-server.model :as model]))

(defn- setup-with-device [device]
  (quil/frame-rate 12)
  (quil/stroke-weight 4)
  (quil/fill (quil/color 200 200 200 20))
  (quil/background 200)
  (quil/set-state! :sequence-number 1))

(defn- draw-needle [x y length angle]
  (quil/push-matrix)
  (quil/translate x y)
  (quil/rotate angle)
  (quil/line 0 0 0 (- length))
  (quil/pop-matrix))

(defn- draw-with-device [device]
  (let [sequence-number (quil/state :sequence-number)]
    (serial/write-series-number device sequence-number)
    (quil/set-state! :sequence-number (inc sequence-number))
    (when-let [telemetry (serial/read-telemetry device)]
      (let [{q :quaternion a :accel} telemetry
            {q-yaw :heading q-pitch :attitude q-roll :bank} (model/quaternion->euler q)
            {a-pitch :attitude a-roll :bank} (model/accel->euler a)
            quarter-width (/ (quil/width) 4)
            third-height (/ (quil/height) 3)
            line-length (/ quarter-width 3)]
        
        (quil/no-stroke)
        (quil/rect 0 0 (quil/width) (quil/height))

        (quil/stroke 0)
        (draw-needle quarter-width third-height line-length q-yaw)
        (draw-needle (* 2 quarter-width) third-height line-length q-pitch)
        (draw-needle (* 3 quarter-width) third-height line-length q-roll)

        (draw-needle (* 2 quarter-width) (* 2 third-height) line-length a-pitch)
        (draw-needle (* 3 quarter-width) (* 2 third-height) line-length a-roll)))))

(defn visualizer [device]
  (quil/defsketch visualizer 
    :title "Magic glove visualizer"
    :setup #(setup-with-device device)
    :draw #(draw-with-device device)
    :size [700 500]))
