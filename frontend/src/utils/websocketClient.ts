// WebSocket client for real-time chat communication
class WebSocketClient {
  private socket: WebSocket | null = null;
  private url: string;
  private reconnectAttempts = 0;
  private maxReconnectAttempts = 3;
  private reconnectInterval = 5000; // 5 seconds
  private messageHandlers: Array<(data: any) => void> = [];
  private isConnecting = false; // Flag to prevent multiple simultaneous connections

  constructor(url: string) {
    this.url = url;
  }

  connect(): Promise<void> {
    // Prevent multiple simultaneous connection attempts
    if (this.isConnecting || (this.socket && this.socket.readyState === WebSocket.CONNECTING)) {
      return Promise.reject(new Error('Connection attempt already in progress'));
    }

    return new Promise((resolve, reject) => {
      try {
        this.isConnecting = true;
        this.socket = new WebSocket(this.url);

        this.socket.onopen = () => {
          console.log('WebSocket connection established');
          this.reconnectAttempts = 0; // Reset reconnect attempts on successful connection
          this.isConnecting = false;
          resolve();
        };

        this.socket.onclose = (event) => {
          console.log(`WebSocket connection closed: ${event.code} ${event.reason}`);
          this.isConnecting = false;

          // Attempt to reconnect if it wasn't a manual close
          if (event.code !== 1000 && this.reconnectAttempts < this.maxReconnectAttempts) {
            console.log(`Attempting to reconnect... (${this.reconnectAttempts + 1}/${this.maxReconnectAttempts})`);
            setTimeout(() => {
              this.reconnectAttempts++;
              this.connect().catch(() => {
                // Don't reject here as we're handling retries internally
                console.log('Reconnection failed, will retry again if attempts remain');
              });
            }, this.reconnectInterval);
          } else if (event.code !== 1000) {
            console.log('Max reconnection attempts reached, giving up');
          }
        };

        this.socket.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data as string);
            // Call all registered message handlers
            this.messageHandlers.forEach(handler => handler(data));
          } catch (error) {
            console.error('Error parsing WebSocket message:', error);
            // Call handlers with raw data if parsing fails
            this.messageHandlers.forEach(handler => handler(event.data));
          }
        };

        this.socket.onerror = (error) => {
          console.error('WebSocket error:', error);
          this.isConnecting = false;
          reject(error);
        };
      } catch (error) {
        console.error('Failed to create WebSocket connection:', error);
        this.isConnecting = false;
        reject(error);
      }
    });
  }

  send(data: any, userId: string | null = null, accessToken: string | null = null): void {
    if (this.socket && this.socket.readyState === WebSocket.OPEN) {
      const payload = { ...data };
      if (userId) {
        payload.user_id = userId;
      }
      if (accessToken) {
        payload.access_token = accessToken;
      }
      this.socket.send(JSON.stringify(payload));
    } else {
      console.error('WebSocket is not connected');
    }
  }

  addMessageHandler(callback: (data: any) => void): void {
    this.messageHandlers.push(callback);
  }

  removeMessageHandler(callback: (data: any) => void): void {
    const index = this.messageHandlers.indexOf(callback);
    if (index !== -1) {
      this.messageHandlers.splice(index, 1);
    }
  }

  clearMessageHandlers(): void {
    this.messageHandlers = [];
  }

  close(): void {
    if (this.socket) {
      this.socket.close(1000, 'Manual close');
      this.socket = null;
      this.isConnecting = false;
      this.messageHandlers = [];
    }
  }

  isConnected(): boolean {
    return this.socket !== null && this.socket.readyState === WebSocket.OPEN;
  }

  isConnectingInProgress(): boolean {
    return this.isConnecting || (this.socket !== null && this.socket.readyState === WebSocket.CONNECTING);
  }
}

// Create a singleton instance
const wsClient = new WebSocketClient('ws://localhost:9000/ws');

export default wsClient;
// TEMPORARILY DISABLED – backend WS unstable
// export default null;
