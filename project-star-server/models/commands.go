/* project-star-server/models/commands.go */

package models

import "time"

// CommandRequest represents a generic command structure
type CommandRequest struct {
	Command    string      `json:"command"`
	Parameters interface{} `json:"parameters,omitempty"`
	DeviceID   string      `json:"device_id"`
}

// CommandResponse represents the response to a command
type CommandResponse struct {
	Status    string      `json:"status"`
	Message   string      `json:"message,omitempty"`
	Data      interface{} `json:"data,omitempty"`
	Timestamp int64       `json:"timestamp"`
}

// NewResponse creates a new command response
func NewResponse(status, message string, data interface{}) CommandResponse {
	return CommandResponse{
		Status:    status,
		Message:   message,
		Data:      data,
		Timestamp: time.Now().Unix(),
	}
}
