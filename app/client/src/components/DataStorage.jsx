/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

import React, { useEffect, useState } from "react";
import Select from "react-select";
import "../styles/StorageSettingsModal.css";

const api = (path) => `${process.env.REACT_APP_API_BASE_URL}${path}`;

const StorageSettingsModal = ({ isOpen, onClose }) => {
  const [isClosing, setIsClosing] = useState(false);
  const [screen, setScreen] = useState(null); // null => MENU, 'create' | 'start' | 'stop' | 'play'
  const [message, setMessage] = useState("");

  // wspólne dane
  const [buckets, setBuckets] = useState([]);
  const [recordedTopics, setRecordedTopics] = useState([]);
  const [currentlyRecordingTopics, setCurrentlyRecordingTopics] = useState([]);

  // create bucket
  const [bucketName, setBucketName] = useState("");
  const [retentionDays, setRetentionDays] = useState(7);
  const [bucketDesc, setBucketDesc] = useState("");

  // start
  const [startBucket, setStartBucket] = useState("");
  const [allTopics, setAllTopics] = useState([]);           // z /api/topics (array stringów: nazwy)
  const [selectedTopics, setSelectedTopics] = useState([]); // zaznaczone do nagrywania (array stringów)

  // stop
  const [stopTopics, setStopTopics] = useState([]);         // array stringów

  // stop playing
  const [currentlyPlayingTopics, setCurrentlyPlayingTopics] = useState([]);
  const [stopPlayingTopics, setStopPlayingTopics] = useState([]);

  // play
  const [playTopic, setPlayTopic] = useState("");           // string
  const [playSessions, setPlaySessions] = useState([]);     // {record_id,bucket_name,label}
  const [playRecordId, setPlayRecordId] = useState("");     // string

  const handleCancel = () => {
    setIsClosing(true);
    setTimeout(() => {
      setIsClosing(false);
      onClose?.();
    }, 600);
  };

  // --- LOADERS ---
  const fetchBuckets = async () => {
    try {
      const r = await fetch(api("/api/get_influx_buckets"));
      const j = await r.json();
      // backend może nie mieć "success" -> defensywnie
      const ok = j.success ?? !j.error_message;
      if (ok) {
        const list = j.influx_buckets || [];
        setBuckets(list);
        if (!startBucket && list.length) setStartBucket(list[0]);
      } else {
        setMessage(j.error_message || "Failed to load buckets.");
      }
    } catch (e) {
      console.error(e);
      setMessage("Network error while loading buckets.");
    }
  };

  const fetchRecordedTopics = async () => {
    try {
      const r = await fetch(api("/api/get_recorded_topics"));
      const j = await r.json();
      const ok = j.success ?? !j.error_message;
      if (ok) setRecordedTopics(j.topics || []);
      else setMessage(j.error_message || "Failed to load recorded topics.");
    } catch (e) {
      console.error(e);
      setMessage("Network error while loading recorded topics.");
    }
  };

  const fetchCurrentlyRecordingTopics = async () => {
    try {
      const r = await fetch(api("/api/get_currently_recording_topics"));
      const j = await r.json();
      const ok = j.success ?? !j.error_message;
      if (ok) setCurrentlyRecordingTopics(j.topics || []);
      else setMessage(j.error_message || "Failed to load currently recording topics.");
    } catch (e) {
      console.error(e);
      setMessage("Network error while loading currently recording topics.");
    }
  };

  const fetchCurrentlyPlayingTopics = async () => {
    try {
      console.log("Fetching currently playing topics...");
      const r = await fetch(api("/api/get_currently_playing_topics"));
      const j = await r.json();
      console.log("Currently playing topics response:", j);
      const ok = j.success ?? !j.error_message;
      if (ok) {
        setCurrentlyPlayingTopics(j.topics || []);
        console.log("Set currently playing topics:", j.topics || []);
        if (screen === "stopPlaying") {
          setMessage(j.topics && j.topics.length > 0 
            ? `Found ${j.topics.length} currently playing topic(s).`
            : "No topics are currently playing.");
        }
      } else {
        setMessage(j.error_message || "Failed to load currently playing topics.");
      }
    } catch (e) {
      console.error("Error fetching currently playing topics:", e);
      setMessage("Network error while loading currently playing topics.");
    }
  };

  const fetchRosTopics = async () => {
    try {
      const r = await fetch(api("/api/topics")); // bez filtrów
      const j = await r.json();
      // Obsłuż 2 formaty: tablica lub { topics: [...] }
      const list = Array.isArray(j) ? j : (Array.isArray(j?.topics) ? j.topics : []);
      const names = list.map((t) => (typeof t === "string" ? t : t?.name)).filter(Boolean);
      setAllTopics(names);
    } catch (e) {
      console.error(e);
      setAllTopics([]);
      setMessage("Network error while loading topics.");
    }
  };

  // init each open
  useEffect(() => {
    if (!isOpen) return;
    setMessage("");
    setScreen(null); // menu
    fetchBuckets();
    fetchRecordedTopics();
    fetchCurrentlyRecordingTopics();
    fetchCurrentlyPlayingTopics();
    // reset
    setSelectedTopics([]);
    setStopTopics([]);
    setStopPlayingTopics([]);
    setPlayTopic("");
    setPlaySessions([]);
    setPlayRecordId("");
  }, [isOpen]);

  // sessions for play
  useEffect(() => {
    const go = async () => {
      setPlaySessions([]);
      setPlayRecordId("");
      if (!playTopic) return;
      try {
        const r = await fetch(api("/api/get_recorded_bags_by_topic"), {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ topic_name: playTopic }),
        });
        const j = await r.json();
        const ok = j.success ?? !j.error;
        if (ok) {
          const sessions = (j.record_ids || []).map((id, i) => ({
            record_id: id,
            bucket_name: j.bucket_names?.[i],
            label: j.end_time_stamps?.[i]
              ? `#${i + 1} – ${j.end_time_stamps[i]}`
              : `#${i + 1} – ${id}`,
          }));
          setPlaySessions(sessions);
          if (sessions[0]) setPlayRecordId(sessions[0].record_id);
        } else {
          setMessage(j.error || j.error_message || "Failed to get sessions.");
        }
      } catch (e) {
        console.error(e);
        setMessage("Network error while loading sessions.");
      }
    };
    go();
  }, [playTopic]);

  // gdy wchodzimy w ekran START → dociągnij tematy i odśwież buckety
  useEffect(() => {
    if (!isOpen || screen !== "start") return;
    fetchBuckets();
    fetchRosTopics();
  }, [isOpen, screen]);

  // gdy wchodzimy w ekran STOP → odśwież obecnie nagrywane tematy
  useEffect(() => {
    if (!isOpen || screen !== "stop") return;
    fetchCurrentlyRecordingTopics();
  }, [isOpen, screen]);

  // gdy wchodzimy w ekran STOP PLAYING → odśwież obecnie odtwarzane tematy
  useEffect(() => {
    if (!isOpen || screen !== "stopPlaying") return;
    console.log("Entering stop playing screen, fetching currently playing topics...");
    setMessage("Loading currently playing topics...");
    fetchCurrentlyPlayingTopics();
  }, [isOpen, screen]);

  if (!isOpen && !isClosing) return null;

  // --- ACTIONS ---
  const handleCreateBucket = async () => {
    setMessage("");
    try {
      const r = await fetch(api("/api/create_bucket"), {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          bucket_name: bucketName,
          retention_days: Number(retentionDays),
          description: bucketDesc,
        }),
      });
      const j = await r.json();
      const ok = j.success ?? !j.error_message;
      if (ok) {
        setMessage("✅ Bucket created.");
        setBucketName("");
        setBucketDesc("");
        fetchBuckets();
      } else setMessage(`❌ ${j.error_message || "Create bucket failed."}`);
    } catch {
      setMessage("❌ Network error while creating bucket.");
    }
  };

  const handleStartRecording = async () => {
    if (!startBucket || selectedTopics.length === 0) {
      setMessage("Wybierz bucket i co najmniej jeden topic.");
      return;
    }
    setMessage("");
    try {
      const r = await fetch(api("/api/start_record_topics"), {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          bucket_name: startBucket,
          topics_names: selectedTopics,
        }),
      });
      const j = await r.json();
      const ok = j.success ?? !j.error;
      if (ok) {
        setMessage("✅ Recording started.");
        setSelectedTopics([]);
        fetchRecordedTopics();
        fetchCurrentlyRecordingTopics();
      } else setMessage("❌ Start recording failed.");
    } catch {
      setMessage("❌ Network error while starting recording.");
    }
  };

  const handleStopRecording = async () => {
    if (stopTopics.length === 0) {
      setMessage("Wybierz co najmniej jeden topic do zatrzymania.");
      return;
    }
    setMessage("");
    try {
      const r = await fetch(api("/api/stop_record_topics"), {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ topics_names: stopTopics }),
      });
      const j = await r.json();
      const ok = j.success ?? !j.error;
      if (ok) {
        setMessage("🛑 Recording stopped.");
        setStopTopics([]);
        fetchRecordedTopics();
        fetchCurrentlyRecordingTopics();
      } else setMessage(`❌ ${j.error || "Stop recording failed."}`);
    } catch {
      setMessage("❌ Network error while stopping recording.");
    }
  };

  const handleStopPlaying = async () => {
    if (stopPlayingTopics.length === 0) {
      setMessage("Wybierz co najmniej jeden topic do zatrzymania odtwarzania.");
      return;
    }
    setMessage("");
    try {
      const r = await fetch(api("/api/stop_playing_topics"), {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ topics_names: stopPlayingTopics }),
      });
      const j = await r.json();
      const ok = j.success ?? !j.error;
      if (ok) {
        setMessage("⏹️ Playback stopped.");
        setStopPlayingTopics([]);
        fetchCurrentlyPlayingTopics();
      } else setMessage(`❌ ${j.error || "Stop playback failed."}`);
    } catch {
      setMessage("❌ Network error while stopping playback.");
    }
  };

  const handlePlayRecording = async () => {
    const sess = playSessions.find((s) => s.record_id === playRecordId);
    if (!playTopic || !sess) {
      setMessage("Wybierz topic i sesję do odtworzenia.");
      return;
    }
    setMessage("");
    try {
      const r = await fetch(api("/api/play_recordings"), {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          bucket_name: sess.bucket_name,
          topic_names: [playTopic],
          record_ids: [sess.record_id],
        }),
      });
      const j = await r.json();
      const ok = j.success ?? !j.error;
      if (ok) setMessage("▶️ Playback started.");
      else setMessage(`❌ ${j.error || "Playback failed."}`);
    } catch {
      setMessage("❌ Network error while starting playback.");
    }
  };

  // ======== RENDER ========

  // MENU (pierwszy ekran)
  if (screen === null) {
    return (
      <div className={`storage-modal ${isClosing ? "closing" : ""}`}>
        <div className={`storage-content ${isClosing ? "closing" : ""}`}>
          <button type="button" onClick={handleCancel} className="storage-close">
            &times;
          </button>

          <h2 className="storage-title">Storage settings</h2>

          <div className="storage-menu">
            <button onClick={() => setScreen("create")} className="storage-btn">
              Create bucket
            </button>
            <button onClick={() => setScreen("start")} className="storage-btn">
              Start recording topics
            </button>
            <button onClick={() => setScreen("stop")} className="storage-btn">
              Stop recording topic
            </button>
            <button onClick={() => setScreen("play")} className="storage-btn">
              Play recorded topic
            </button>
            <button onClick={() => setScreen("stopPlaying")} className="storage-btn">
              Stop playing topic
            </button>
          </div>

          {message && <div className="storage-msg">{message}</div>}
        </div>
      </div>
    );
  }

  // CREATE BUCKET
  if (screen === "create") {
    return (
      <div className={`create-action-modal ${isClosing ? "closing" : ""}`}>
        <div className={`modal-content ${isClosing ? "closing" : ""}`}>
          <h2>Create bucket</h2>
          {message && <div className="message">{message}</div>}
          <form className="new-action-form" onSubmit={(e) => e.preventDefault()}>
            <div className="form-group">
              <label>Bucket name:</label>
              <input
                value={bucketName}
                onChange={(e) => setBucketName(e.target.value)}
                type="text"
                required
              />
            </div>
            <div className="form-group">
              <label>Retention (days):</label>
              <input
                value={retentionDays}
                onChange={(e) => setRetentionDays(e.target.value)}
                type="number"
                min="1"
              />
            </div>
            <div className="form-group">
              <label>Description:</label>
              <input
                value={bucketDesc}
                onChange={(e) => setBucketDesc(e.target.value)}
                type="text"
              />
            </div>
            <div className="modal-actions">
              <button type="button" className="add-button" onClick={handleCreateBucket}>
                Create
              </button>
              <button type="button" onClick={handleCancel} className="close-button">
                Cancel
              </button>
              <button type="button" onClick={() => setScreen(null)} className="back-button">
                Back
              </button>
            </div>
          </form>
        </div>
      </div>
    );
  }

  // START RECORDING (react-select)
  if (screen === "start") {
    // opcje do react-select
    const bucketOptions = buckets.map((b) => ({ value: b, label: b }));
    const topicOptions  = allTopics.map((t) => ({ value: t, label: t }));

    return (
      <div className={`create-action-modal ${isClosing ? "closing" : ""}`}>
        <div className={`modal-content ${isClosing ? "closing" : ""}`}>
          <h2>Start recording topics</h2>
          {message && <div className="message">{message}</div>}
          <form className="new-action-form" onSubmit={(e) => e.preventDefault()}>
            <div className="form-group">
              <label>Bucket:</label>
              <Select
                className="multi-select"
                classNamePrefix="my-select"
                options={bucketOptions}
                value={bucketOptions.find((o) => o.value === startBucket) || null}
                onChange={(opt) => setStartBucket(opt?.value || "")}
                placeholder="Select bucket..."
                isClearable
              />
            </div>

            <div className="form-group">
              <label>Topics (multi-select):</label>
              <Select
                className="multi-select"
                classNamePrefix="my-select"
                isMulti
                options={topicOptions}
                value={topicOptions.filter((o) => selectedTopics.includes(o.value))}
                onChange={(selected) => setSelectedTopics((selected || []).map((o) => o.value))}
                placeholder="Select topics..."
                closeMenuOnSelect={false}
              />
            </div>

            <div className="modal-actions">
              <button type="button" className="add-button" onClick={handleStartRecording}>
                Start
              </button>
              <button type="button" onClick={handleCancel} className="close-button">
                Cancel
              </button>
              <button type="button" onClick={() => setScreen(null)} className="back-button">
                Back
              </button>
            </div>
          </form>
        </div>
      </div>
    );
  }

  // STOP RECORDING (react-select isMulti)
  if (screen === "stop") {
    const currentlyRecordingOptions = currentlyRecordingTopics.map((t) => ({ value: t, label: t }));

    return (
      <div className={`create-action-modal ${isClosing ? "closing" : ""}`}>
        <div className={`modal-content ${isClosing ? "closing" : ""}`}>
          <h2>Stop recording topic</h2>
          {message && <div className="message">{message}</div>}
          <form className="new-action-form" onSubmit={(e) => e.preventDefault()}>
            <div className="form-group">
              <label>Currently recording topics:</label>
              <Select
                className="multi-select"
                classNamePrefix="my-select"
                isMulti
                options={currentlyRecordingOptions}
                value={currentlyRecordingOptions.filter((o) => stopTopics.includes(o.value))}
                onChange={(selected) => setStopTopics((selected || []).map((o) => o.value))}
                placeholder="Select topics to stop..."
                closeMenuOnSelect={false}
                noOptionsMessage={() => "(no currently recording topics)"}
              />
            </div>
            <div className="modal-actions">
              <button
                type="button"
                className="add-button"
                style={{ background: "linear-gradient(145deg, #ff625a, #e0463f)" }}
                onClick={handleStopRecording}
              >
                Stop
              </button>
              <button type="button" onClick={handleCancel} className="close-button">
                Cancel
              </button>
              <button type="button" onClick={() => setScreen(null)} className="back-button">
                Back
              </button>
            </div>
          </form>
        </div>
      </div>
    );
  }

  // STOP PLAYING (react-select isMulti)
  if (screen === "stopPlaying") {
    const currentlyPlayingOptions = currentlyPlayingTopics.map((t) => ({ value: t, label: t }));

    return (
      <div className={`create-action-modal ${isClosing ? "closing" : ""}`}>
        <div className={`modal-content ${isClosing ? "closing" : ""}`}>
          <h2>Stop playing topic</h2>
          {message && <div className="message">{message}</div>}
          <form className="new-action-form" onSubmit={(e) => e.preventDefault()}>
            <div className="form-group">
              <label>Currently playing topics:</label>
              <Select
                className="multi-select"
                classNamePrefix="my-select"
                isMulti
                options={currentlyPlayingOptions}
                value={currentlyPlayingOptions.filter((o) => stopPlayingTopics.includes(o.value))}
                onChange={(selected) => setStopPlayingTopics((selected || []).map((o) => o.value))}
                placeholder="Select topics to stop playing..."
                closeMenuOnSelect={false}
                noOptionsMessage={() => "(no currently playing topics)"}
              />
            </div>
            <div className="modal-actions">
              <button
                type="button"
                className="add-button"
                style={{ background: "linear-gradient(145deg, #ff8c42, #e07738)" }}
                onClick={handleStopPlaying}
              >
                Stop Playing
              </button>
              <button type="button" onClick={handleCancel} className="close-button">
                Cancel
              </button>
              <button type="button" onClick={() => setScreen(null)} className="back-button">
                Back
              </button>
            </div>
          </form>
        </div>
      </div>
    );
  }

  // PLAY (react-select)
  {
    const topicOptions   = recordedTopics.map((t) => ({ value: t, label: t }));
    const sessionOptions = playSessions.map((s) => ({
      value: s.record_id,
      label: `${s.label} – ${s.bucket_name}`,
    }));

    return (
      <div className={`create-action-modal ${isClosing ? "closing" : ""}`}>
        <div className={`modal-content ${isClosing ? "closing" : ""}`}>
          <h2>Play recorded topic</h2>
          {message && <div className="message">{message}</div>}
          <form className="new-action-form" onSubmit={(e) => e.preventDefault()}>
            <div className="form-group">
              <label>Topic:</label>
              <Select
                className="multi-select"
                classNamePrefix="my-select"
                options={topicOptions}
                value={topicOptions.find((o) => o.value === playTopic) || null}
                onChange={(opt) => setPlayTopic(opt?.value || "")}
                placeholder="Select topic…"
                isClearable
              />
            </div>
            <div className="form-group">
              <label>Session:</label>
              <Select
                className="multi-select"
                classNamePrefix="my-select"
                options={sessionOptions}
                value={sessionOptions.find((o) => o.value === playRecordId) || null}
                onChange={(opt) => setPlayRecordId(opt?.value || "")}
                placeholder={playTopic ? "Select session…" : "(select topic first)"}
                isDisabled={!playTopic || sessionOptions.length === 0}
                isClearable
                noOptionsMessage={() => "(no sessions)"}
              />
            </div>
            <div className="modal-actions">
              <button
                type="button"
                className="add-button"
                disabled={!playTopic || !playRecordId}
                onClick={handlePlayRecording}
              >
                Play
              </button>
              <button type="button" onClick={handleCancel} className="close-button">
                Cancel
              </button>
              <button type="button" onClick={() => setScreen(null)} className="back-button">
                Back
              </button>
            </div>
          </form>
        </div>
      </div>
    );
  }
};

export default StorageSettingsModal;