(ns glove-server.serial
  (:require [serial-port :as serial]
            [clojure.string :as string]
            [clojure.core.async :as async])
  (:import (java.io InputStream)))

;; Not happy with this. Waiting for the clojure-foo to sink into my brain.
;; WISHLIST
;;   * Use rxtx22 directly rather than serial-port
;;   * Better tests
;;   * Record dropped bytes


(defn read-16-bit-int [^InputStream in-stream]
  (let [high-byte (.read in-stream)
        low-byte (.read in-stream)]
    (BigInteger. (byte-array [high-byte low-byte]))))

(defn qscale [x] (/ (double x) 32767.0))

;; TOO SLOW? Why don't channels work with this?
(defn attempt-read-record
  [out-chan ^InputStream in-stream]
  (when (and (>= (.available in-stream) 46)
             (= (byte \H) (.read in-stream)))
    (let [thumb (read-16-bit-int in-stream)
          index (read-16-bit-int in-stream)
          middle (read-16-bit-int in-stream)
          ring (read-16-bit-int in-stream)
          pinky (read-16-bit-int in-stream)
          require-newline (.read in-stream)
          require-A (.read in-stream)]
      (when (and (= (byte \newline) require-newline)
                 (= (byte \A) require-A))
        (let [accel-x (read-16-bit-int in-stream)
              accel-y (read-16-bit-int in-stream)
              accel-z (read-16-bit-int in-stream)
              require-newline (.read in-stream)
              require-G (.read in-stream)]
          (when (and (= (byte \newline) require-newline)
                     (= (byte \G) require-G))
            (let [gyro-x (read-16-bit-int in-stream)
                  gyro-y (read-16-bit-int in-stream)
                  gyro-z (read-16-bit-int in-stream)
                  require-newline (.read in-stream)
                  require-M (.read in-stream)]
              (when (and (= (byte \newline) require-newline)
                         (= (byte \M) require-M))
                (let [mag-x (read-16-bit-int in-stream)
                      mag-y (read-16-bit-int in-stream)
                      mag-z (read-16-bit-int in-stream)
                      require-newline (.read in-stream)
                      require-Q (.read in-stream)]
                  (when (and (= (byte \newline) require-newline)
                             (= (byte \Q) require-Q))
                    (let [q-0-i (read-16-bit-int in-stream)
                          q-1-i (read-16-bit-int in-stream)
                          q-2-i (read-16-bit-int in-stream)
                          q-3-i (read-16-bit-int in-stream)
                          require-newline (.read in-stream)]
                      (when (= (byte \newline) require-newline)
                        (let [q-0 (qscale q-0-i)
                              q-1 (qscale q-1-i)
                              q-2 (qscale q-2-i)
                              q-3 (qscale q-3-i)
                              ret {:fingers [thumb index middle ring pinky]
                                   :accel [accel-x accel-y accel-z]
                                   :mag [mag-x mag-y mag-z]
                                   :quaternion [q-0 q-1 q-2 q-3]}]
                          (println ret) ;; LAG HERE FUCKS IT ALL UP
                          (comment "Cant figure out how to actually write this to a channel"))))))))))))))

(defn read-from-port [port from-device-chan]
  (async/thread
   (serial/listen port #(attempt-read-record from-device-chan %) true)))

(defn write-to-port [port to-device-chan]
  (async/go-loop []
           (let [bytes (async/<! to-device-chan)]
             (when-not (nil? bytes)
               (serial/write port bytes)
               (recur)))))

(defn connect-device [device-name]
  "Connects to a serial port and begins to process the serial input.
   Returns a device: {:in channel :out channel}"
  (let [port (serial/open device-name 115200)
        from-device-chan (async/chan (async/sliding-buffer 5))
        to-device-chan (async/chan)]
    (async/thread
     (serial/listen port #(attempt-read-record from-device-chan %)))
    (write-to-port port to-device-chan)
    {:from-device from-device-chan :to-device to-device-chan :port port}))

(defn close-device [{:keys [port to-device from-device]}]
  (serial/close port)
  (async/close! to-device)
  (async/close! from-device))

(defn read-channel
  "returns a channel of newline-separated lines from the device"
  [{:keys [from-device] :as device}]
  from-device)

(defn write-bytes-channel
  "returns a channel that writes byte arrays to device."
  [{:keys [to-device]}]
  to-device)

(comment
  (use 'glove-server.serial :reload)
  (require '[clojure.core.async :as async])
  (def device (connect-device "/dev/tty.usbserial-FTE3RR3T"))
  (def device-read (read-channel device))
  (def device-write (write-bytes-channel device))
  (async/<!! device-read)
  (async/>!! write-chan (byte-array [127 127 127 10]))
)
