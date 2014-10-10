(ns glove-server.serial
  (:require [serial-port :as serial]
            [clojure.core.async :refer [chan close! go go-loop <! >! >!! <!! thread]]))


(defn read-from-port [port from-device-chan]
  (thread
   (serial/on-byte port #(>!! from-device-chan %))))

(defn write-to-port [port to-device-chan]
  (go-loop []
           (let [bytes (<! to-device-chan)]
             (when-not (nil? bytes)
               (serial/write port bytes)
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

(def ^:const line-max-length 70)

(defn read-lines-channel
  "returns a channel of newline-separated lines from the device"
  [{:keys [from-device]}]
  (let [lines-out (chan)]
    (go-loop [s [] line-length-left line-max-length]
             (let [next-char (char (<! from-device))]
               (cond

                (nil? next-char)
                (do (>! lines-out (apply str s))
                    (close! lines-out))

                (or (= next-char \newline) (= line-length-left 0))
                (do (>! lines-out (apply str s))
                    (recur [] line-max-length))

                :else
                (recur (conj s next-char) (dec line-length-left)))))
    lines-out))

(defn write-bytes-channel
  "returns a channel that writes byte arrays to device."
  [{:keys [to-device]}]
  to-device)

(comment
  (use 'glove-server.serial)
  (require `[clojure.core.async :as async])
  (def device (connect-device "/dev/tty.usbserial-FTE3RR3T"))
  (def device-read (read-lines-channel device))
  (def device-write (write-bytes-channel device))
  (async/<!! device-read)
  (async/>!! write-chan (byte-array [127 127 127 10]))
)