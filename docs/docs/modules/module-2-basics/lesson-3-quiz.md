---
sidebar_position: 3
---

# Lesson 3: Module 2 Quiz

## Test Your Understanding

This quiz will assess your understanding of the concepts covered in Module 2: Basics. Please answer the following questions based on what you've learned.

### Question 1: Modularity Benefits

Which of the following are benefits of modularity? (Select all that apply)

- A) Easier to understand individual components
- B) Components can be reused in different contexts
- C) Requires less documentation
- D) Individual components can be tested independently

<details>
  <summary>Answer</summary>
  
  A, B, and D are correct. Modularity makes individual components easier to understand, allows for component reuse, and enables independent testing. Modularity doesn't inherently require less documentation.
</details>

### Question 2: Abstraction Purpose

What is the main purpose of abstraction in system design?

- A) To make systems run faster
- B) To simplify complex systems by hiding implementation details
- C) To reduce memory usage
- D) To ensure thread safety

<details>
  <summary>Answer</summary>
  
  B is correct. Abstraction simplifies complex systems by focusing on essential features while hiding implementation details.
</details>

### Question 3: Practical Application

In the e-commerce example, what was the purpose of the `validateOrder` function?

- A) To calculate the total cost of the order
- B) To check if the order contains necessary information
- C) To process the customer's payment
- D) To update inventory levels

<details>
  <summary>Answer</summary>
  
  B is correct. The `validateOrder` function checked if the order had required fields like items and customerId.
</details>

### Question 4: System Components

Which principle guided the breaking down of the order processing system into functions like `validateOrder`, `calculateTotal`, and `processPayment`?

- A) Scalability
- B) Iteration
- C) Modularity
- D) Optimization

<details>
  <summary>Answer</summary>
  
  C is correct. The principle of modularity guided breaking the system into distinct, manageable components.
</details>

## Practical Exercise

Modify the order processing example to include a discount calculation component. The discount should apply if the order total exceeds $100, providing a 10% discount.

Continue to [Module 3: Advanced Topics](../module-3-advanced/lesson-1-advanced-topics.md) to explore more sophisticated applications of these principles.