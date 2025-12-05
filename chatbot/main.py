import os
import asyncio
from agents import Agent, Runner, OpenAIChatCompletionsModel, AsyncOpenAI,RunConfig
from dotenv import load_dotenv
from pydantic import BaseModel
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

load_dotenv()
API_KEY = os.getenv("GEMINI_API_KEY")

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],   # For development only
    allow_credentials=True,
    allow_methods=["*"],   # ✅ This enables OPTIONS
    allow_headers=["*"],
)
client = AsyncOpenAI(
    api_key=API_KEY,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)
model = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",
    openai_client=client,
)
config = RunConfig(
    model=model,
    model_provider=client,
    tracing_disabled=True,
)
agent = Agent(
    name="Assistant",
    instructions="You are a helpful Assistant",
)
class AskRequest(BaseModel):
    message : str
@app.post("/ask")
async def rag_chatbot(req:AskRequest):
    result = await Runner.run(agent, input=req.message,run_config=config )
    return result.final_output


# to run
# uv run uvicorn main:app