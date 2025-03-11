package main

import (
	"log"
	"net/http"
	"os"

	"github.com/Topographic-Robot/Project-STAR/server/api"
	"github.com/Topographic-Robot/Project-STAR/server/device"
)

func main() {
	// Get configuration from environment variables with defaults
	port := getEnv("PORT", "8080")
	deviceAddr := getEnv("DEVICE_ADDR", "192.168.1.100:80")

	log.Printf("Starting Project STAR API server on port %s", port)
	log.Printf("Connecting to device at %s", deviceAddr)

	// Initialize device connector
	connector := device.New(deviceAddr)

	// Initialize API handlers
	handler := api.New(connector)

	// Register routes
	http.HandleFunc("/api/movement", handler.MovementHandler)
	http.HandleFunc("/api/sensors", handler.SensorsHandler)
	http.HandleFunc("/api/expansions", handler.ExpansionsHandler)
	http.HandleFunc("/api/status", handler.StatusHandler)

	// Start server
	log.Printf("Server listening on :%s", port)
	if err := http.ListenAndServe(":"+port, nil); err != nil {
		log.Fatalf("Server failed to start: %v", err)
	}
}

// getEnv gets an environment variable or returns a default value
func getEnv(key, defaultValue string) string {
	value := os.Getenv(key)
	if value == "" {
		return defaultValue
	}
	return value
}
