/* project-star-server/device/connector.go */

package device

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"log"
	"net/http"
	"sync"
	"time"
)

// Command represents a command to be sent to the ESP32 device
type Command struct {
	Type       string      `json:"type"`
	Action     string      `json:"action"`
	Parameters interface{} `json:"parameters,omitempty"`
	Timestamp  int64       `json:"timestamp"`
}

// Connector manages communication with ESP32 devices
type Connector struct {
	client       *http.Client
	deviceAddr   string
	commandQueue []Command
	mutex        sync.Mutex
}

// New creates a new device connector
func New(deviceAddr string) *Connector {
	connector := &Connector{
		client: &http.Client{
			Timeout: 5 * time.Second,
		},
		deviceAddr:   deviceAddr,
		commandQueue: make([]Command, 0),
	}

	// Start a goroutine to process the command queue periodically
	go func() {
		ticker := time.NewTicker(5 * time.Second)
		defer ticker.Stop()

		for range ticker.C {
			connector.ProcessCommandQueue()
		}
	}()

	return connector
}

// SendCommand sends a command to the device
func (c *Connector) SendCommand(cmdType, action string, params interface{}) error {
	cmd := Command{
		Type:       cmdType,
		Action:     action,
		Parameters: params,
		Timestamp:  time.Now().Unix(),
	}

	err := c.sendCommandToDevice(cmd)
	if err != nil {
		c.mutex.Lock()
		c.commandQueue = append(c.commandQueue, cmd)
		c.mutex.Unlock()
		log.Printf("Command queued: %s - %s", cmdType, action)
	}

	return err
}

func (c *Connector) sendCommandToDevice(cmd Command) error {
	jsonData, err := json.Marshal(cmd)
	if err != nil {
		return err
	}

	url := "http://" + c.deviceAddr + "/api/command"
	resp, err := c.client.Post(url, "application/json", bytes.NewBuffer(jsonData))
	if err != nil {
		return err
	}
	defer resp.Body.Close()

	if resp.StatusCode != http.StatusOK {
		bodyBytes, _ := io.ReadAll(resp.Body) // Read the response body
		return fmt.Errorf("device returned error status: %s, body: %s", resp.Status, string(bodyBytes))
	}
	return nil
}

// ProcessCommandQueue attempts to send queued commands
func (c *Connector) ProcessCommandQueue() {
	c.mutex.Lock()
	defer c.mutex.Unlock()

	if len(c.commandQueue) == 0 {
		return
	}

	log.Printf("Processing command queue: %d commands", len(c.commandQueue))
	var remainingCommands []Command

	for _, cmd := range c.commandQueue {
		if err := c.sendCommandToDevice(cmd); err != nil {
			remainingCommands = append(remainingCommands, cmd)
		} else {
			log.Printf("Queued command processed: %s - %s", cmd.Type, cmd.Action)
		}
	}

	c.commandQueue = remainingCommands
}

// GetQueueLength returns the number of queued commands
func (c *Connector) GetQueueLength() int {
	c.mutex.Lock()
	defer c.mutex.Unlock()
	return len(c.commandQueue)
}
