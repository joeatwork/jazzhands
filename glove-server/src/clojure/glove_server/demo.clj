(ns glove-server.demo
  (:use overtone.live)
  (:require [glove-server.serial :as serial]
            [glove-server.model :as model]))

;; Copied wholesale from the overtone example directory at
;; https://github.com/overtone/overtone/blob/master/src/overtone/examples/instruments/dubstep.clj
(defsynth dubstep [bpm 120 wobble 1 note 50 snare-vol 1 kick-vol 1 v 1 out-bus 0]
 (let [trig (impulse:kr (/ bpm 120))
       freq (midicps note)
       swr (demand trig 0 (dseq [wobble] INF))
       sweep (lin-exp (lf-tri swr) -1 1 40 3000)
       wob (apply + (saw (* freq [0.99 1.01])))
       wob (lpf wob sweep)
       wob (* 0.8 (normalizer wob))
       wob (+ wob (bpf wob 1500 2))
       wob (+ wob (* 0.2 (g-verb wob 9 0.7 0.7)))

       kickenv (decay (t2a (demand (impulse:kr (/ bpm 30)) 0 (dseq [1 0 0 0 0 0 1 0 1 0 0 1 0 0 0 0] INF))) 0.7)
       kick (* (* kickenv 7) (sin-osc (+ 40 (* kickenv kickenv kickenv 200))))
       kick (clip2 kick 1)

       snare (* 3 (pink-noise) (apply + (* (decay (impulse (/ bpm 240) 0.5) [0.4 2]) [1 0.05])))
       snare (+ snare (bpf (* 4 snare) 2000))
       snare (clip2 snare 1)]

   (out out-bus    (* v (clip2 (+ wob (* kick-vol kick) (* snare-vol snare)) 1)))))

(def notes [25 27 28 35 40 41 50])

(defn ctl-dubstep [device synth]
  (loop [msg-number 1]
    (serial/write-series-number device msg-number)
    (when-let [telemetry (serial/read-telemetry device)]
      (let [{a :accel m :mag} telemetry
            {:keys [bank attitude heading]} (model/accel->mag->euler a m)
            wobs (int (* 10 (Math/abs bank)))
            note-ix (mod (int (* 8 (Math/abs attitude))) (count notes))
            note (notes note-ix)]
        (println "msg" msg-number "wobs" wobs "note" note-ix "bank" bank "attitude" attitude "heading" heading)
        (ctl synth :wobble wobs)
        (ctl synth :note note)))
    (recur (inc msg-number))))
