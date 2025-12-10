# Data Model: Physical AI & Humanoid Robotics Book

## Entities

### Learning Module
- **ID**: Unique identifier for the module
- **Title**: Name of the module (e.g., "The Robotic Nervous System (ROS 2)")
- **Description**: Brief overview of the module's focus
- **Lessons**: Collection of lesson IDs belonging to this module
- **Order**: Numeric position in the sequence of modules (1-4)
- **Learning Objectives**: List of skills/knowledge students will gain
- **Prerequisites**: Modules/lessons that should be completed before starting

### Lesson
- **ID**: Unique identifier for the lesson
- **ModuleID**: Reference to the parent module
- **Title**: Name of the lesson (e.g., "Introduction to ROS 2 Nodes")
- **Content**: Markdown content of the lesson
- **CodeExamples**: List of code example IDs
- **Exercises**: List of exercise IDs
- **LearningObjectives**: Specific objectives for this lesson
- **Duration**: Estimated time to complete (in minutes)
- **Order**: Numeric position in the sequence of lessons within the module (1-4)

### Code Example
- **ID**: Unique identifier for the code example
- **Language**: Programming language (Python, C++, etc.)
- **Content**: Source code text
- **Description**: Explanation of what the code does
- **LessonID**: Reference to the lesson that contains this example
- **FilePath**: Relative path where code is stored in the project

### Exercise
- **ID**: Unique identifier for the exercise
- **Title**: Name of the exercise
- **Description**: Detailed explanation of the exercise
- **LessonID**: Reference to the lesson that contains this exercise
- **Type**: 'practical', 'theoretical', 'quiz', etc.
- **Difficulty**: 'beginner', 'intermediate'
- **ExpectedOutcome**: What the student should achieve
- **Solution**: Sample solution or guidance
- **Resources**: Additional files or references needed

### User Progress
- **UserID**: Identifier for the user (if implemented)
- **LessonID**: Reference to the lesson being tracked
- **Status**: 'not-started', 'in-progress', 'completed'
- **CompletionDate**: When the lesson was completed
- **AssessmentScore**: Score from any quizzes/exercises

## Relationships

1. Module (1) -> (4) Lessons: Each module contains exactly 4 lessons
2. Lesson (1) -> (n) Code Examples: Each lesson can have multiple code examples
3. Lesson (1) -> (n) Exercises: Each lesson can have multiple exercises
4. User Progress (n) -> (1) Lesson: Multiple user progress records for each lesson

## Validation Rules

1. Each module must have exactly 4 lessons
2. Each lesson must have at least one code example
3. Each lesson must have at least one hands-on exercise (as per constitution)
4. Module order must be sequential (1, 2, 3, 4)
5. Lesson order within a module must be sequential (1, 2, 3, 4)
6. All code examples must have a specified language
7. Learning objectives must be specified for each module and lesson
8. Estimated duration must be provided for each lesson

## State Transitions

### Lesson Status
- 'not-started' -> 'in-progress': When user begins the lesson
- 'in-progress' -> 'completed': When user completes all exercises/quizzes
- 'completed' -> 'in-progress': When user chooses to review the lesson