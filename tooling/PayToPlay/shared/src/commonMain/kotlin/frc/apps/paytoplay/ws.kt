package frc.apps.paytoplay

import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.mutableStateOf
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.wannaverse.websockets.WebSocketManager
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.catch
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlin.time.Clock
import kotlin.time.Duration.Companion.milliseconds

enum class ConnectionStatus {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    ERROR
}

class ws : ViewModel() {

    val websocketURL = mutableStateOf("ws://localhost:5000")

    private var wsManager: WebSocketManager? = null

    val connectionStatus = mutableStateOf(ConnectionStatus.DISCONNECTED)
    val lastError = mutableStateOf<String?>(null)
    val lastHeartbeatMs = mutableStateOf<Long>(0)
    val messages = mutableStateListOf<String>()

    private var autoReconnectJob: Job? = null
    private var incomingJob: Job? = null

    fun updateUrl(newUrl: String) {
        if (websocketURL.value != newUrl) {
            websocketURL.value = newUrl
            if (connectionStatus.value == ConnectionStatus.CONNECTED) {
                disconnect()
            }
        }
    }

    fun connectToServer() = viewModelScope.launch {
        if (connectionStatus.value == ConnectionStatus.CONNECTING || connectionStatus.value == ConnectionStatus.CONNECTED) {
            return@launch
        }

        connectionStatus.value = ConnectionStatus.CONNECTING
        lastError.value = null

        try {
            val manager = WebSocketManager(websocketURL.value)
            wsManager = manager

            manager.connect()

            incomingJob?.cancel()
            incomingJob = manager.incomingMessages
                .onEach { message ->
                    println("Received: $message")
                    messages.add(0, "Server: $message")
                    lastHeartbeatMs.value = Clock.System.now().toEpochMilliseconds()
                }
                .catch { e ->
                    println("WebSocket error: ${e.message}")
                    connectionStatus.value = ConnectionStatus.ERROR
                    lastError.value = e.message ?: "Stream error"
                }
                .launchIn(viewModelScope)

            connectionStatus.value = ConnectionStatus.CONNECTED
            lastError.value = null
        } catch (e: Throwable) {
            println("WebSocket connection error: ${e.message}")
            connectionStatus.value = ConnectionStatus.ERROR
            lastError.value = e.message ?: "Failed to connect to ${websocketURL.value}"
            try {
                wsManager?.disconnect()
            } catch (_: Throwable) {}
        }
    }

    fun sendMessage(message: String): Boolean {
        return try {
            if (connectionStatus.value != ConnectionStatus.CONNECTED || wsManager == null) {
                messages.add(0, "Error: Failed to send - Not connected")
                return false
            }
            messages.add(0, "Client: $message")
            wsManager?.send(message)
            true
        } catch (e: Throwable) {
            connectionStatus.value = ConnectionStatus.ERROR
            lastError.value = e.message ?: "Failed to send message"
            messages.add(0, "Error sending: ${e.message}")
            false
        }
    }

    fun startAutoReconnect() {
        if (autoReconnectJob?.isActive == true) return
        autoReconnectJob = viewModelScope.launch {
            while (isActive) {
                if (connectionStatus.value != ConnectionStatus.CONNECTED && connectionStatus.value != ConnectionStatus.CONNECTING) {
                    connectToServer()
                }
                delay(3000.milliseconds)
            }
        }
    }

    fun stopAutoReconnect() {
        autoReconnectJob?.cancel()
        autoReconnectJob = null
    }

    fun disconnect() {
        stopAutoReconnect()
        try {
            wsManager?.disconnect()
        } catch (_: Throwable) {}
        connectionStatus.value = ConnectionStatus.DISCONNECTED
    }

    fun clearLog() {
        messages.clear()
    }

    fun isConnected(): Boolean {

        if (lastHeartbeatMs.value > 200) connectionStatus.value = ConnectionStatus.DISCONNECTED

        return connectionStatus.value == ConnectionStatus.CONNECTED
    }

    override fun onCleared() {
        disconnect()
        try {
            wsManager?.clear()
        } catch (_: Throwable) {}
        super.onCleared()
    }
}
