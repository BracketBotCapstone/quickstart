<script lang="ts">
  import { createRoom, getMessages, id, transactMessage } from '$lib/services/db';
  
  const { useTypingIndicator } = createRoom('chat', 'main');
  const typing = useTypingIndicator('chat');
  let messageText = $state('');

  let query = getMessages();

  function typingInfo(users: any[]) {
    if (users.length === 0) return null;
    if (users.length === 1) return `${users[0].name} is typing...`;
    return `${users[0].name} and ${users.length - 1} others are typing...`;
  }

  function handleKeyPress(event: KeyboardEvent) {
    if (event.key === 'Enter' && !event.shiftKey) {
      console.log('enter key pressed');
      event.preventDefault();
      if (messageText.trim()) {
        addMessage(messageText.trim());
      }
    }
  }

  const addMessage = (text: string) => {
    console.log('adding message', text);
    const messageId = id();
    transactMessage({ id: messageId, data: { text, createdAt: Date.now() } });
    messageText = '';
  };

  interface Message {
    text: string;
    timestamp: number;
  }
</script>

{#if query.isLoading}
  <div>Loading...</div>
{:else if query.error}
  <div>Error: {query.error.message}</div>
{:else}
  <div class="messages">
    {#each Object.values(query.data.messages || {}) as message}
      <div class="message">
        {(message as unknown as Message).text}
      </div>
    {/each}
  </div>

  <div class="input-area">
    <textarea
      
      bind:value={messageText}
      onkeydown={handleKeyPress}
      placeholder="Type a message..."
      class="message-input"
    ></textarea>
    <div class="typing-indicator">
      {typing.active.length ? typingInfo(typing.active) : ''}
    </div>
  </div>
{/if}

<style>
  .messages {
    height: 300px;
    overflow-y: auto;
    border: 1px solid #ccc;
    margin-bottom: 1rem;
    padding: 1rem;
  }

  .message-input {
    width: 100%;
    height: 80px;
    padding: 0.5rem;
  }

  .typing-indicator {
    font-size: 0.8rem;
    color: #666;
    height: 1.2em;
  }
</style> 
