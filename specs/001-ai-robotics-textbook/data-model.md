# Data Model: AI-Native Textbook on Physical AI & Humanoid Robotics

**Feature Branch**: `001-ai-robotics-textbook`  
**Created**: December 4, 2025  
**Spec**: [specs/001-ai-robotics-textbook/spec.md](specs/001-ai-robotics-textbook/spec.md)
**Plan**: [specs/001-ai-robotics-textbook/plan.md](specs/001-ai-robotics-textbook/plan.md)
**Research**: [specs/001-ai-robotics-textbook/research.md](specs/001-ai-robotics-textbook/research.md)

## Key Entities

### Student

Represents a learner interacting with the textbook and the RAG chatbot.

*   **Attributes**:
    *   `student_id` (UUID): Unique identifier for the student.
    *   `programming_level` (Enum: 'beginner', 'intermediate', 'advanced'): Self-declared programming proficiency.
    *   `ai_knowledge_level` (Enum: 'none', 'basic', 'intermediate', 'advanced'): Self-declared AI knowledge.
    *   `hardware_availability` (Enum: 'local_gpu', 'no_gpu', 'cloud_access'): Information about the student's computing resources.
    *   `learning_speed_preference` (Enum: 'fast', 'moderate', 'slow'): How quickly the student prefers to progress.
    *   `session_history` (JSONB): Stores a summary of past interactions and learning progress.
    *   `interaction_logs` (Array of LogEntry): Detailed log of chatbot interactions.
    *   `personalization_settings` (JSONB): Stores preferences for content adaptation.
*   **Relationships**:
    *   One-to-many with `Query` (a student can make many queries).
    *   One-to-many with `Assessment` (a student can take many assessments).
    *   One-to-many with `TextbookContent` (for tracking reading progress).
*   **Validation Rules**: `student_id` must be unique.

### TextbookContent

Represents a logical unit of the textbook, such as a chapter or a section within a chapter.

*   **Attributes**:
    *   `content_id` (UUID): Unique identifier for the content unit.
    *   `chapter_id` (UUID): Identifier for the parent chapter.
    *   `section_id` (String): Identifier for the specific section (e.g., "1.2.3").
    *   `title` (String): Title of the chapter/section.
    *   `markdown_content` (Text): The full Markdown source of the content.
    *   `code_examples` (Array of CodeExample): Embedded code snippets.
    *   `lab_instructions` (Text): Step-by-step instructions for labs.
    *   `learning_objectives` (Array of String): List of learning objectives.
    *   `assessments` (Array of AssessmentRef): References to related assessments.
    *   `version_metadata` (String): Version hash or timestamp for content versioning.
*   **Relationships**:
    *   One-to-many with `VectorEmbedding` (a content unit can be chunked into many embeddings).
*   **Validation Rules**: `content_id` and `section_id` must be unique within their scope.

### Query

Represents a user's input to the RAG chatbot.

*   **Attributes**:
    *   `query_id` (UUID): Unique identifier for the query.
    *   `student_id` (UUID): ID of the student who made the query.
    *   `query_text` (Text): The actual text of the query.
    *   `timestamp` (Timestamp): Time when the query was made.
    *   `context` (JSONB): Additional context like selected text, current chapter, etc.
*   **Relationships**:
    *   Many-to-one with `Student`.
    *   One-to-one with `Response`.
*   **Validation Rules**: `query_id` must be unique.

### Response

Represents the RAG chatbot's output to a user query.

*   **Attributes**:
    *   `response_id` (UUID): Unique identifier for the response.
    *   `query_id` (UUID): ID of the query this response is for.
    *   `response_text` (Text): The chatbot's generated answer.
    *   `timestamp` (Timestamp): Time when the response was generated.
    *   `citation_chapter_id` (UUID): ID of the cited chapter.
    *   `citation_section_id` (String): ID of the cited section.
    *   `retrieval_trace` (JSONB): Details of the retrieval process (e.g., chunks retrieved, scores).
    *   `source_chunk_ids` (Array of UUID): IDs of the specific text chunks used for retrieval.
    *   `agent_type` (Enum: 'retrieval', 'verification', 'explanation', 'assessment', 'safety'): The primary agent responsible for the response.
    *   `refusal_reason` (String): If the query was refused, the reason why.
*   **Relationships**:
    *   One-to-one with `Query`.
*   **Validation Rules**: `response_id` must be unique.

### VectorEmbedding

Represents a numerical embedding of a chunk of textbook content. Stored in Qdrant.

*   **Attributes**:
    *   `embedding_id` (UUID): Unique identifier for the embedding.
    *   `vector` (Array of Float): The embedding vector.
    *   `text_chunk` (Text): The original text chunk.
    *   `content_id` (UUID): Reference to the `TextbookContent` it originated from.
    *   `chapter_id` (UUID): Reference to the parent chapter.
    *   `section_id` (String): Reference to the parent section.
    *   `start_line`, `end_line` (Integer): Line range of the chunk in the original Markdown.
    *   `version_metadata` (String): Version of the textbook content it was derived from.
*   **Relationships**:
    *   Many-to-one with `TextbookContent`.
*   **Validation Rules**: `embedding_id` must be unique.

### Assessment

Represents a quiz, exercise, or mini-project related to textbook content.

*   **Attributes**:
    *   `assessment_id` (UUID): Unique identifier for the assessment.
    *   `chapter_id` (UUID): The chapter this assessment belongs to.
    *   `type` (Enum: 'quiz', 'exercise', 'mini-project'): Type of assessment.
    *   `question_text` (Text): The question or prompt for the assessment.
    *   `options` (Array of String): For multiple-choice questions.
    *   `correct_answer` (Text/JSONB): The correct answer or criteria for evaluation.
    *   `student_responses` (Array of StudentAssessmentResponse): History of student attempts.
*   **Relationships**:
    *   Many-to-one with `TextbookContent`.
*   **Validation Rules**: `assessment_id` must be unique.

### AgentConfiguration

Represents the configuration for each type of AI agent.

*   **Attributes**:
    *   `agent_type` (Enum: 'retrieval', 'verification', 'explanation', 'assessment', 'safety'): Type of agent.
    *   `model_name` (String): The specific LLM model used (e.g., 'gemini-1.5-flash').
    *   `prompt_template` (Text): The base prompt for the agent.
    *   `tools_available` (Array of String): List of tools the agent can use.
    *   `memory_scope` (Enum: 'session', 'chapter', 'global'): How long the agent retains memory.
    *   `fallback_behavior` (String): Instructions for deterministic fallback.
    *   `version` (String): Version of the agent configuration.
*   **Validation Rules**: `agent_type` must be unique.
