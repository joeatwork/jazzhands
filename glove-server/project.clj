(defproject glove-server "0.1.0-SNAPSHOT"
  :description "Tools for Joe's Dataglove"
  :url "http://www.joe-bowers.com"
  :license {:name "Eclipse Public License"
            :url "http://www.eclipse.org/legal/epl-v10.html"}
  :source-paths ["src/clojure"]
  :java-source-paths ["src/java"]
  :dependencies [[org.clojure/clojure "1.6.0"]
                 [org.clojure/core.async "0.1.346.0-17112a-alpha"]
                 [org.clojure/algo.monads "0.1.5"]
                 [overtone "0.9.1"]
                 [serial-port "1.1.2"]
                 [quil "2.2.2"]])
