(ns glove-server.diagnostics
  (:require [glove-server.serial :as serial]))

(defn read-message-number [device wait-for-num]
  (loop []
    (let [{num :message :as telemetry} (serial/read-telemetry device)]
      (if (= num wait-for-num)
        telemetry
        (recur)))))

(defn calculate-model-noise)

(defn collect-telemetry [device trials]
  (loop [collected []]
    (let [series-number (inc (count collected))]
      (if (< series-number trials)
        (do
          (serial/write-series-number device series-number)
          (let [telemetry (read-message-number device series-number)]
            (recur (conj collected telemetry))))
        collected))))
