(ns glove-server.serial
  (:require [serial-port :as serial]
            [clojure.core.async :refer [chan close! go go-loop <! >! >!! <!! thread]]))

(defn read-from-port [port from-device-chan]
  (thread
   (serial/on-byte port #(>!! from-device-chan %))))

(defn write-to-port [port to-device-chan]
  (go-loop []
           (let [byte (<! to-device-chan)]
             (when (nil? byte)
               (serial/write port)
               (recur)))))

(defn connect-device [device-name]
  "Connects to a serial port and begins to process the serial input.
   Returns a device: {:in channel :out channel}"
  (let [port (serial/open device-name 115200)
        from-device-chan (chan 128)
        to-device-chan (chan)]
    (read-from-port port from-device-chan)
    (write-to-port port to-device-chan)
    {:from-device from-device-chan :to-device to-device-chan :port port}))

(defn close-device [{:keys [port to-device from-device]}]
  (serial/close port)
  (close! to-device)
  (close! from-device))

(defn print-line [{:keys [from-device]}]
  (loop [s [] limit 70]
    (let [char (char (<!! from-device))]
      (if (or (= char \newline) (= limit 0))
        (println (apply str s))
        (recur (conj s char) (dec limit))))))

