"use client"

import { useState, useEffect, JSX } from "react"
import { createBot } from "botui"
import { BotUI, BotUIMessageList, BotUIAction } from "@botui/react"
import "../../css/botui.css" // Make sure this exists
const myBot: any = createBot()

interface Message {
  text: string
  human?: boolean
}

export default function BookChatbot(): JSX.Element {
  const [open, setOpen] = useState(false)
  const [messages, setMessages] = useState<Message[]>([])
  const [input, setInput] = useState("")

  // Add a message to bot and local state
  const addMessage = async (text: string, human = false) => {
    await myBot.message.add({ text, human })
    setMessages((prev) => [...prev, { text, human }])
  }

  // Handle sending a user question
  const handleSubmit = async () => {
    if (!input.trim()) return
    await addMessage(input, true)

    try {
      const res = await fetch("/api/ask", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ message: input }),
      })
      const data = await res.json()
      await addMessage(data.answer || "Sorry, no response.")
    } catch (err) {
      await addMessage("Error fetching answer. Try again later.")
      console.error(err)
    }

    setInput("")
  }

  // Initial greeting when chatbot opens
  useEffect(() => {
    if (!open) return
    addMessage("Hi 👋 I’m your AI assistant for this book!")
  }, [open])

  return (
    <>
      {/* Floating Chat Button */}
      <button
        onClick={() => setOpen(!open)}
        style={{
          position: "fixed",
          bottom: "24px",
          right: "24px",
          zIndex: 9999,
          background: "#2563eb",
          color: "white",
          borderRadius: "999px",
          width: "60px",
          height: "60px",
          fontSize: "22px",
          border: "none",
          cursor: "pointer",
          boxShadow: "0 10px 25px rgba(0,0,0,0.3)",
        }}
      >
        🤖
      </button>

      {/* Chat Window */}
      {open && (
        <div
          style={{
            position: "fixed",
            bottom: "96px",
            right: "24px",
            width: "350px",
            height: "500px",
            background: "white",
            borderRadius: "16px",
            overflow: "hidden",
            zIndex: 9999,
            boxShadow: "0 20px 40px rgba(0,0,0,0.35)",
            display: "flex",
            flexDirection: "column",
          }}
        >
          {/* Header */}
          <div
            style={{
              background: "#2563eb",
              color: "white",
              padding: "12px 16px",
              fontWeight: "bold",
              display: "flex",
              justifyContent: "space-between",
              alignItems: "center",
            }}
          >
            📘 Book AI Assistant
            <button
              onClick={() => setOpen(false)}
              style={{
                background: "transparent",
                border: "none",
                color: "white",
                fontSize: "18px",
                cursor: "pointer",
              }}
            >
              ✕
            </button>
          </div>

          {/* BotUI Body */}
            <div style={{ flex: 1, display: "flex", flexDirection: "column" }}>
            <BotUI bot={myBot}>
                <BotUIMessageList />
            </BotUI>

            {/* Input outside BotUIAction */}
            <div style={{ display: "flex", marginTop: "auto", padding: "8px" }}>
                <input
                type="text"
                value={input}
                onChange={(e) => setInput(e.target.value)}
                placeholder="Type your question..."
                style={{ flex: 1, padding: "8px", borderRadius: "8px 0 0 8px", border: "1px solid #ccc" }}
                onKeyDown={(e) => e.key === "Enter" && handleSubmit()}
                />
                <button
                onClick={handleSubmit}
                style={{ padding: "8px 12px", borderRadius: "0 8px 8px 0", background: "#2563eb", color: "white" }}
                >
                Send
                </button>
            </div>
            </div>

        </div>
      )}
    </>
  )
}
