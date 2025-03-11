package api

import (
	"encoding/json"
	"fmt"
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

// genericCommandHandler handles commands of various types
func (h *Handler) genericCommandHandler(w http.ResponseWriter, r *http.Request, commandType string) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var cmd models.CommandRequest
	err := json.NewDecoder(r.Body).Decode(&cmd)
	if err != nil {
		log.Printf("Error decoding request body: %v", err)
		http.Error(w, "Invalid JSON format: "+err.Error(), http.StatusBadRequest)
		return
	}

	// Check for empty command
	if cmd.Command == "" {
		http.Error(w, "Missing required field: command", http.StatusBadRequest)
		return
	}

	log.Printf("Received %s command: %s", commandType, cmd.Command)

	err = h.connector.SendCommand(commandType, cmd.Command, cmd.Parameters)
	if err != nil {
		log.Printf("Error sending command: %v", err)
	}

	response := models.CommandResponse{
		Status:    "success",
		Message:   fmt.Sprintf("%s command processed", commandType),
		Timestamp: time.Now().Unix(),
	}

	if err != nil {
		response.Status = "queued"
		response.Message = "Command queued for later delivery"
	}

	w.Header().Set("Content-Type", "application/json")
	if err := json.NewEncoder(w).Encode(response); err != nil {
		log.Printf("Error encoding response: %v", err)
		http.Error(w, "Internal Server Error", http.StatusInternalServerError)
		return
	}
}

// MovementHandler handles movement commands
func (h *Handler) MovementHandler(w http.ResponseWriter, r *http.Request) {
	h.genericCommandHandler(w, r, "movement")
}

// SensorsHandler handles sensor commands
func (h *Handler) SensorsHandler(w http.ResponseWriter, r *http.Request) {
	h.genericCommandHandler(w, r, "sensor")
}

// ExpansionsHandler handles expansion commands
func (h *Handler) ExpansionsHandler(w http.ResponseWriter, r *http.Request) {
	h.genericCommandHandler(w, r, "expansion")
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
	if err := json.NewEncoder(w).Encode(response); err != nil {
		log.Printf("Error encoding response: %v", err)
		http.Error(w, "Internal Server Error", http.StatusInternalServerError)
		return
	}
}
