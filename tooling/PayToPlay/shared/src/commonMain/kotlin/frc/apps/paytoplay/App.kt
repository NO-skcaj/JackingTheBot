package frc.apps.paytoplay

import androidx.compose.animation.AnimatedVisibility
import androidx.compose.animation.fadeIn
import androidx.compose.animation.fadeOut
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.atruedev.kmpnfc.adapter.NfcAdapter
import com.atruedev.kmpnfc.reader.AndroidScanMode
import com.atruedev.kmpnfc.reader.ReaderOptions

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun App() {
    MaterialTheme(
        colorScheme = darkColorScheme(
            primary = Color(0xFF8AB4F8),        // Google Blue (Dark mode)
            onPrimary = Color(0xFF003062),
            primaryContainer = Color(0xFF00468A),
            onPrimaryContainer = Color(0xFFD6E3FF),
            secondary = Color(0xFFF28B82),      // FRC 3824 Accent / Coral
            secondaryContainer = Color(0xFF7C2E2B),
            onSecondaryContainer = Color(0xFFFFDAD6),
            surface = Color(0xFF1F1F1F),
            surfaceVariant = Color(0xFF2D2D2D),
            background = Color(0xFF121212),
            onSurface = Color(0xFFE3E2E6),
            onSurfaceVariant = Color(0xFFC4C6D0),
            error = Color(0xFFF2B8B5),
            errorContainer = Color(0xFF8C1D18)
        )
    ) {
        val adapter = remember { NfcAdapter() }
        val connection = remember { ws() }

        var numOfTags by remember { mutableStateOf(0) }
        var tagData by remember { mutableStateOf("Ready to scan") }
        var isScanning by remember { mutableStateOf(false) }

        var serverUrlInput by remember { mutableStateOf(connection.websocketURL.value) }

        // Manage NFC Scanning & Server Auto-reconnect
        LaunchedEffect(isScanning) {
            if (!isScanning) {
                connection.stopAutoReconnect()
                return@LaunchedEffect
            }

            // Start auto-reconnection loop to remote server while scanning is active
            connection.isConnected()
            try {
                if (!connection.isConnected()) connection.startAutoReconnect()
            } catch (e: Exception) {
                try {
                    connection.stopAutoReconnect()
                } catch (e: Exception) {
                    println(e)
                }
            }
            tagData = "Hold NFC tag near phone..."

            try {
                adapter.tags(
                    ReaderOptions(androidScanMode = AndroidScanMode.ReaderMode)
                ).collect { tag ->
                    val result = tag.use {
                        it.hashCode().toString()
                    }

                    numOfTags++
                    tagData = "Tag ID: $result"
                    connection.sendMessage(numOfTags.toString())
                }
            } catch (e: Exception) {
                tagData = "NFC Error: ${e.message ?: "Failed to read tag"}"
                isScanning = false
                connection.stopAutoReconnect()
            }
        }

        Scaffold(
            topBar = {
                Surface(
                    tonalElevation = 3.dp,
                    color = MaterialTheme.colorScheme.surfaceVariant
                ) {
                    Column(
                        modifier = Modifier
                            .fillMaxWidth()
                            .padding(horizontal = 20.dp, vertical = 14.dp)
                    ) {
                        Row(
                            verticalAlignment = Alignment.CenterVertically,
                            horizontalArrangement = Arrangement.SpaceBetween,
                            modifier = Modifier.fillMaxWidth()
                        ) {
                            Column {
                                Row(verticalAlignment = Alignment.CenterVertically) {
                                    Surface(
                                        color = MaterialTheme.colorScheme.primaryContainer,
                                        shape = RoundedCornerShape(6.dp),
                                        modifier = Modifier.padding(end = 8.dp)
                                    ) {
                                        Text(
                                            text = "FRC 3824",
                                            style = MaterialTheme.typography.labelSmall,
                                            fontWeight = FontWeight.Bold,
                                            color = MaterialTheme.colorScheme.onPrimaryContainer,
                                            modifier = Modifier.padding(horizontal = 6.dp, vertical = 2.dp)
                                        )
                                    }
                                    Text(
                                        text = "PayToPlay",
                                        style = MaterialTheme.typography.titleLarge,
                                        fontWeight = FontWeight.Bold,
                                        color = MaterialTheme.colorScheme.onSurface
                                    )
                                }
                                Text(
                                    text = "NFC Payment & Event Management",
                                    style = MaterialTheme.typography.bodySmall,
                                    color = MaterialTheme.colorScheme.onSurfaceVariant
                                )
                            }

                            // Connection Status Chip
                            ConnectionChip(status = connection.connectionStatus.value)
                        }
                    }
                }
            },
            containerColor = MaterialTheme.colorScheme.background
        ) { paddingValues ->
            LazyColumn(
                modifier = Modifier
                    .fillMaxSize()
                    .padding(paddingValues)
                    .padding(horizontal = 16.dp),
                verticalArrangement = Arrangement.spacedBy(16.dp),
                contentPadding = PaddingValues(vertical = 16.dp)
            ) {
                // 1. NFC Scanner Controller Card
                item {
                    ElevatedCard(
                        modifier = Modifier.fillMaxWidth(),
                        shape = RoundedCornerShape(20.dp),
                        colors = CardDefaults.elevatedCardColors(
                            containerColor = MaterialTheme.colorScheme.surface
                        )
                    ) {
                        Column(
                            modifier = Modifier
                                .fillMaxWidth()
                                .padding(20.dp),
                            horizontalAlignment = Alignment.CenterHorizontally
                        ) {
                            Text(
                                text = "NFC Scanner Control",
                                style = MaterialTheme.typography.titleMedium,
                                fontWeight = FontWeight.SemiBold,
                                color = MaterialTheme.colorScheme.onSurface,
                                modifier = Modifier.align(Alignment.Start)
                            )

                            Spacer(modifier = Modifier.height(16.dp))

                            // Large Interactive Scanner Toggle Button
                            Button(
                                onClick = { isScanning = !isScanning },
                                modifier = Modifier
                                    .fillMaxWidth()
                                    .height(56.dp),
                                shape = RoundedCornerShape(16.dp),
                                colors = ButtonDefaults.buttonColors(
                                    containerColor = if (isScanning) MaterialTheme.colorScheme.errorContainer else MaterialTheme.colorScheme.primaryContainer,
                                    contentColor = if (isScanning) MaterialTheme.colorScheme.error else MaterialTheme.colorScheme.onPrimaryContainer
                                )
                            ) {
                                Text(
                                    text = if (isScanning) "Stop Scanning" else "Start Scanning",
                                    style = MaterialTheme.typography.titleMedium,
                                    fontWeight = FontWeight.Bold
                                )
                            }

                            Spacer(modifier = Modifier.height(14.dp))

                            // Dynamic Info Badge / Auto Sync Indicator
                            Surface(
                                shape = RoundedCornerShape(12.dp),
                                color = if (isScanning) MaterialTheme.colorScheme.primaryContainer.copy(alpha = 0.4f) else MaterialTheme.colorScheme.surfaceVariant,
                                modifier = Modifier.fillMaxWidth()
                            ) {
                                Row(
                                    modifier = Modifier.padding(horizontal = 12.dp, vertical = 8.dp),
                                    verticalAlignment = Alignment.CenterVertically
                                ) {
                                    Box(
                                        modifier = Modifier
                                            .size(8.dp)
                                            .clip(CircleShape)
                                            .background(if (isScanning) Color(0xFF4CAF50) else Color.Gray)
                                    )
                                    Spacer(modifier = Modifier.width(8.dp))
                                    Text(
                                        text = if (isScanning) "Auto-reconnect & scanning active" else "Scanner idle (Auto-sync paused)",
                                        style = MaterialTheme.typography.bodySmall,
                                        color = MaterialTheme.colorScheme.onSurfaceVariant
                                    )
                                }
                            }
                        }
                    }
                }

                // 2. NFC Live Status & Tag Counter
                item {
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.spacedBy(12.dp)
                    ) {
                        // Tags Counter Card
                        OutlinedCard(
                            modifier = Modifier.weight(1f),
                            shape = RoundedCornerShape(16.dp)
                        ) {
                            Column(
                                modifier = Modifier.padding(16.dp),
                                horizontalAlignment = Alignment.Start
                            ) {
                                Text(
                                    text = "Tags Scanned",
                                    style = MaterialTheme.typography.labelMedium,
                                    color = MaterialTheme.colorScheme.onSurfaceVariant
                                )
                                Spacer(modifier = Modifier.height(4.dp))
                                Text(
                                    text = "$numOfTags",
                                    style = MaterialTheme.typography.headlineLarge,
                                    fontWeight = FontWeight.Bold,
                                    color = MaterialTheme.colorScheme.primary
                                )
                            }
                        }

                        // Hardware NFC State Card
                        OutlinedCard(
                            modifier = Modifier.weight(1.3f),
                            shape = RoundedCornerShape(16.dp)
                        ) {
                            Column(
                                modifier = Modifier.padding(16.dp),
                                horizontalAlignment = Alignment.Start
                            ) {
                                Text(
                                    text = "NFC Adapter State",
                                    style = MaterialTheme.typography.labelMedium,
                                    color = MaterialTheme.colorScheme.onSurfaceVariant
                                )
                                Spacer(modifier = Modifier.height(6.dp))
                                Text(
                                    text = adapter.state.value.name,
                                    style = MaterialTheme.typography.titleMedium,
                                    fontWeight = FontWeight.SemiBold,
                                    color = MaterialTheme.colorScheme.onSurface,
                                    maxLines = 1,
                                    overflow = TextOverflow.Ellipsis
                                )
                            }
                        }
                    }
                }

                // 3. Current Tag Reading Output Card
                item {
                    ElevatedCard(
                        modifier = Modifier.fillMaxWidth(),
                        shape = RoundedCornerShape(16.dp),
                        colors = CardDefaults.elevatedCardColors(
                            containerColor = MaterialTheme.colorScheme.surface
                        )
                    ) {
                        Column(modifier = Modifier.padding(16.dp)) {
                            Text(
                                text = "Current Tag Payload",
                                style = MaterialTheme.typography.labelMedium,
                                color = MaterialTheme.colorScheme.onSurfaceVariant
                            )
                            Spacer(modifier = Modifier.height(6.dp))
                            Surface(
                                shape = RoundedCornerShape(10.dp),
                                color = MaterialTheme.colorScheme.surfaceVariant,
                                modifier = Modifier.fillMaxWidth()
                            ) {
                                Text(
                                    text = tagData,
                                    style = MaterialTheme.typography.bodyMedium,
                                    fontWeight = FontWeight.Medium,
                                    color = MaterialTheme.colorScheme.onSurface,
                                    modifier = Modifier.padding(12.dp)
                                )
                            }
                        }
                    }
                }

                // 4. Server Configuration & Error Display Card
                item {
                    ElevatedCard(
                        modifier = Modifier.fillMaxWidth(),
                        shape = RoundedCornerShape(16.dp),
                        colors = CardDefaults.elevatedCardColors(
                            containerColor = MaterialTheme.colorScheme.surface
                        )
                    ) {
                        Column(modifier = Modifier.padding(16.dp)) {
                            Text(
                                text = "Server Configuration",
                                style = MaterialTheme.typography.titleSmall,
                                fontWeight = FontWeight.SemiBold,
                                color = MaterialTheme.colorScheme.onSurface
                            )
                            Spacer(modifier = Modifier.height(10.dp))

                            OutlinedTextField(
                                value = serverUrlInput,
                                onValueChange = {
                                    serverUrlInput = it
                                    connection.updateUrl(it)
                                },
                                label = { Text("WebSocket Server URL") },
                                singleLine = true,
                                modifier = Modifier.fillMaxWidth(),
                                shape = RoundedCornerShape(12.dp)
                            )

                            Spacer(modifier = Modifier.height(12.dp))

                            Row(
                                modifier = Modifier.fillMaxWidth(),
                                horizontalArrangement = Arrangement.spacedBy(8.dp)
                            ) {
                                if (connection.connectionStatus.value == ConnectionStatus.CONNECTED) {
                                    OutlinedButton(
                                        onClick = { connection.disconnect() },
                                        modifier = Modifier.weight(1f),
                                        shape = RoundedCornerShape(12.dp)
                                    ) {
                                        Text("Disconnect")
                                    }
                                } else {
                                    Button(
                                        onClick = { connection.connectToServer() },
                                        modifier = Modifier.weight(1f),
                                        shape = RoundedCornerShape(12.dp),
                                        enabled = connection.connectionStatus.value != ConnectionStatus.CONNECTING
                                    ) {
                                        Text(if (connection.connectionStatus.value == ConnectionStatus.CONNECTING) "Connecting..." else "Connect")
                                    }
                                }

                                FilledTonalButton(
                                    onClick = { connection.sendMessage(numOfTags.toString()) },
                                    modifier = Modifier.weight(1f),
                                    shape = RoundedCornerShape(12.dp),
                                    enabled = connection.connectionStatus.value == ConnectionStatus.CONNECTED
                                ) {
                                    Text("Send Count")
                                }
                            }

                            // Error Banner Display
                            AnimatedVisibility(
                                visible = connection.lastError.value != null,
                                enter = fadeIn(),
                                exit = fadeOut()
                            ) {
                                Surface(
                                    color = MaterialTheme.colorScheme.errorContainer,
                                    shape = RoundedCornerShape(10.dp),
                                    modifier = Modifier
                                        .fillMaxWidth()
                                        .padding(top = 12.dp)
                                ) {
                                    Text(
                                        text = connection.lastError.value ?: "",
                                        style = MaterialTheme.typography.bodySmall,
                                        color = MaterialTheme.colorScheme.error,
                                        modifier = Modifier.padding(10.dp)
                                    )
                                }
                            }
                        }
                    }
                }

                // 5. Activity & WebSocket Log Feed
                item {
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Text(
                            text = "Activity Log",
                            style = MaterialTheme.typography.titleMedium,
                            fontWeight = FontWeight.SemiBold,
                            color = MaterialTheme.colorScheme.onSurface
                        )

                        if (connection.messages.isNotEmpty()) {
                            TextButton(onClick = { connection.clearLog() }) {
                                Text("Clear", style = MaterialTheme.typography.labelMedium)
                            }
                        }
                    }
                }

                if (connection.messages.isEmpty()) {
                    item {
                        Surface(
                            shape = RoundedCornerShape(12.dp),
                            color = MaterialTheme.colorScheme.surfaceVariant.copy(alpha = 0.5f),
                            modifier = Modifier.fillMaxWidth()
                        ) {
                            Text(
                                text = "No network messages yet.",
                                style = MaterialTheme.typography.bodyMedium,
                                color = MaterialTheme.colorScheme.onSurfaceVariant,
                                textAlign = TextAlign.Center,
                                modifier = Modifier.padding(24.dp)
                            )
                        }
                    }
                } else {
                    items(connection.messages) { msg ->
                        LogMessageCard(message = msg)
                    }
                }
            }
        }
    }
}

@Composable
fun ConnectionChip(status: ConnectionStatus) {
    val (bgColor, textColor, label) = when (status) {
        ConnectionStatus.CONNECTED -> Triple(Color(0xFF1E3A1E), Color(0xFF81C784), "Connected")
        ConnectionStatus.CONNECTING -> Triple(Color(0xFF3E2723), Color(0xFFFFB74D), "Connecting")
        ConnectionStatus.ERROR -> Triple(Color(0xFF3C1618), Color(0xFFE57373), "Error")
        ConnectionStatus.DISCONNECTED -> Triple(Color(0xFF2A2A2A), Color(0xFF9E9E9E), "Offline")
    }

    Surface(
        color = bgColor,
        shape = RoundedCornerShape(20.dp)
    ) {
        Row(
            verticalAlignment = Alignment.CenterVertically,
            modifier = Modifier.padding(horizontal = 10.dp, vertical = 4.dp)
        ) {
            Box(
                modifier = Modifier
                    .size(8.dp)
                    .clip(CircleShape)
                    .background(textColor)
            )
            Spacer(modifier = Modifier.width(6.dp))
            Text(
                text = label,
                style = MaterialTheme.typography.labelSmall,
                fontWeight = FontWeight.Bold,
                color = textColor
            )
        }
    }
}

@Composable
fun LogMessageCard(message: String) {
    val isClient = message.startsWith("Client:")
    val isError = message.startsWith("Error:")
    val isServer = message.startsWith("Server:")

    val chipColor = when {
        isError -> MaterialTheme.colorScheme.errorContainer
        isClient -> MaterialTheme.colorScheme.primaryContainer
        else -> MaterialTheme.colorScheme.surfaceVariant
    }

    val textColor = when {
        isError -> MaterialTheme.colorScheme.error
        isClient -> MaterialTheme.colorScheme.onPrimaryContainer
        else -> MaterialTheme.colorScheme.onSurface
    }

    OutlinedCard(
        modifier = Modifier.fillMaxWidth(),
        shape = RoundedCornerShape(12.dp)
    ) {
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(12.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Surface(
                color = chipColor,
                shape = RoundedCornerShape(6.dp),
                modifier = Modifier.padding(end = 10.dp)
            ) {
                Text(
                    text = when {
                        isError -> "ERR"
                        isClient -> "OUT"
                        isServer -> "IN"
                        else -> "LOG"
                    },
                    style = MaterialTheme.typography.labelSmall,
                    fontWeight = FontWeight.Bold,
                    color = textColor,
                    modifier = Modifier.padding(horizontal = 6.dp, vertical = 2.dp)
                )
            }

            Text(
                text = message,
                style = MaterialTheme.typography.bodyMedium,
                color = MaterialTheme.colorScheme.onSurface,
                modifier = Modifier.weight(1f)
            )
        }
    }
}