package api

import (
	"encoding/json"
	"log"
	"net/http"
	"time"

	"github.com/Topographic-Robot/Project-STAR/server/device"
	"github.com/Topographic-Robot/Project-STAR/server/models"
)

// Handler manages API endpoints
type Handler struct {
	connector *device.Connector
}

// New creates a new API handler
func New(connector *device.Connector) *Handler {
	return &Handler{
		connector: connector,
	}
}

// MovementHandler handles movement commands
func (h *Handler) MovementHandler(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var cmd models.CommandRequest
	if err := json.NewDecoder(r.Body).Decode(&cmd); err != nil {
		http.Error(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	log.Printf("Received movement command: %s", cmd.Command)

	err := h.connector.SendCommand("movement", cmd.Command, cmd.Parameters)

	response := models.CommandResponse{
		Status:    "success",
		Message:   "Movement command processed",
		Timestamp: time.Now().Unix(),
	}

	if err != nil {
		response.Status = "queued"
		response.Message = "Command queued for later delivery"
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(response)
}

// SensorsHandler handles sensor commands
func (h *Handler) SensorsHandler(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var cmd models.CommandRequest
	if err := json.NewDecoder(r.Body).Decode(&cmd); err != nil {
		http.Error(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	log.Printf("Received sensor command: %s", cmd.Command)

	err := h.connector.SendCommand("sensor", cmd.Command, cmd.Parameters)

	response := models.CommandResponse{
		Status:    "success",
		Message:   "Sensor command processed",
		Timestamp: time.Now().Unix(),
	}

	if err != nil {
		response.Status = "queued"
		response.Message = "Command queued for later delivery"
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(response)
}

// ExpansionsHandler handles expansion commands
func (h *Handler) ExpansionsHandler(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var cmd models.CommandRequest
	if err := json.NewDecoder(r.Body).Decode(&cmd); err != nil {
		http.Error(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	log.Printf("Received expansion command: %s", cmd.Command)

	err := h.connector.SendCommand("expansion", cmd.Command, cmd.Parameters)

	response := models.CommandResponse{
		Status:    "success",
		Message:   "Expansion command processed",
		Timestamp: time.Now().Unix(),
	}

	if err != nil {
		response.Status = "queued"
		response.Message = "Command queued for later delivery"
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(response)
}

// StatusHandler handles status requests
func (h *Handler) StatusHandler(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	queueLength := h.connector.GetQueueLength()

	response := models.CommandResponse{
		Status:  "success",
		Message: "System operational",
		Data: map[string]interface{}{
			"queued_commands": queueLength,
			"server_time":     time.Now().Format(time.RFC3339),
		},
		Timestamp: time.Now().Unix(),
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(response)
}
