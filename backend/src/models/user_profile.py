from typing import Optional, List, Dict, Any
from uuid import UUID, uuid4
from pydantic import BaseModel, Field

# Enums for profile attributes
class ProgrammingLevel(str, BaseModel):
    beginner = "beginner"
    intermediate = "intermediate"
    advanced = "advanced"

class AIKnowledgeLevel(str, BaseModel):
    none = "none"
    basic = "basic"
    intermediate = "intermediate"
    advanced = "advanced"

class HardwareAvailability(str, BaseModel):
    local_gpu = "local_gpu"
    no_gpu = "no_gpu"
    cloud_access = "cloud_access"

class LearningSpeedPreference(str, BaseModel):
    fast = "fast"
    moderate = "moderate"
    slow = "slow"

class StudentProfile(BaseModel):
    """
    Schema for storing student learning profiles for personalization.
    Corresponds to the 'Student' entity in the data model.
    """
    student_id: UUID = Field(default_factory=uuid4, description="Unique identifier for the student.")
    programming_level: Optional[ProgrammingLevel] = Field(None, description="Self-declared programming proficiency.")
    ai_knowledge_level: Optional[AIKnowledgeLevel] = Field(None, description="Self-declared AI knowledge level.")
    hardware_availability: Optional[HardwareAvailability] = Field(None, description="Information about the student's computing resources.")
    learning_speed_preference: Optional[LearningSpeedPreference] = Field(None, description="How quickly the student prefers to progress.")
    
    # These fields might be managed by other services or directly in the DB as JSONB/Array
    # For now, we keep them as Optional[Dict/List] to reflect the data model
    session_history: Optional[Dict[str, Any]] = Field(None, description="Summary of past interactions and learning progress.")
    interaction_logs: Optional[List[Dict[str, Any]]] = Field(None, description="Detailed log of chatbot interactions.")
    personalization_settings: Optional[Dict[str, Any]] = Field(None, description="Stores specific personalization settings.")

    class Config:
        json_schema_extra = {
            "example": {
                "student_id": "123e4567-e89b-12d3-a456-426614174000",
                "programming_level": "intermediate",
                "ai_knowledge_level": "basic",
                "hardware_availability": "local_gpu",
                "learning_speed_preference": "moderate",
                "session_history": {},
                "interaction_logs": [],
                "personalization_settings": {"explanation_depth": "medium"}
            }
        }
