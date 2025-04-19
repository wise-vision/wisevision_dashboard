# 🧭 WiseVision Dashboard – Copilot Instructions

## 📌 Overview

This document provides best practices and development guidelines for the WiseVision ROS 2 Dashboard frontend, built with Next.js. It aims to ensure a consistent, scalable, and secure development process across all components of the project.

---

## 🔧 Project Structure

- **Use the `app/` Directory**: Leverage Next.js's `app/` directory for routing and layout management.
- **Organize Code into Feature-Based Folders**:
  - `components/`: Reusable UI components.
  - `lib/`: Utility functions and shared logic.
  - `hooks/`: Custom React hooks.
  - `styles/`: Global and component-specific styles.
  - `types/`: TypeScript type definitions.
- **Use Route Groups**: Utilize parentheses in folder names (e.g., `(admin)`) to group related routes without affecting the URL structure.

---

## 🛠️ TypeScript and Linting

- **Enable TypeScript**: Ensure type safety and enhance developer experience.
- **Configure ESLint and Prettier**: Maintain code consistency and catch potential issues early.
- **Strict Mode**: Enable React's Strict Mode to identify potential problems in the application.

---

## ⚙️ State Management

- **Global State**: Use React Context for managing global state where simplicity is key.
- **Local Component State**: Utilize `useState` or `useReducer` for component-specific states.
- **Advanced State Libraries**: For more complex needs, consider lightweight libraries like Zustand or Redux.

---

## 📡 Data Fetching

- **Server-Side Rendering (SSR)**: Use `getServerSideProps` for pages that require dynamic data on each request.
- **Static Site Generation (SSG)**: Use `getStaticProps` for pages with data that can be fetched at build time.
- **Client-Side Fetching**: Use SWR or React Query for client-side data fetching with caching and revalidation.
- **Page-Level Data Fetching**: Avoid fetching data in `_app.tsx` to prevent unnecessary data loading across pages.

---

## 🎨 Styling

- **CSS Modules**: Scope styles locally to components to prevent style conflicts.
- **Global Styles**: Define global styles in `styles/globals.css` and import them in `_app.tsx`.
- **Utility-First CSS**: Consider using Tailwind CSS for rapid UI development with a utility-first approach.

---

## 🚀 Performance Optimization

- **Code Splitting**: Leverage dynamic imports to split code and load components as needed.
- **Image Optimization**: Utilize Next.js's `next/image` component for automatic image optimization.
- **Font Optimization**: Use `next/font` to manage and optimize fonts effectively.
- **Lazy Loading**: Implement lazy loading for heavy components and images to improve initial load times.

---

## 🔐 Security

- **Environment Variables**: Store sensitive information in environment variables and ensure they are not exposed to the client.
- **Content Security Policy (CSP)**: Implement CSP headers to mitigate cross-site scripting (XSS) attacks.
- **HTTPS**: Serve the application over HTTPS to ensure data security in transit.

---

## 🧪 Testing

- **Unit Testing**: Use Jest for testing individual components and functions.
- **Integration Testing**: Use React Testing Library to test component interactions and user flows.
- **End-to-End Testing**: Use Cypress or Playwright for testing full user journeys.
- **Continuous Integration**: Set up CI pipelines to automatically run tests on every commit or pull request.

---

## 📦 Deployment

- **Docker**: Containerize the application using Docker for consistent deployment environments.
- **Snap Packages**: Provide Snap packages for easy installation on supported Linux distributions.
- **Edge Deployment**: Optimize the application for deployment on edge servers with limited resources.
- **Configuration Management**: Implement robust configuration management to handle various deployment scenarios.

---

## 📈 Monitoring and Analytics

- **Error Monitoring**: Integrate tools like Sentry to capture and monitor production errors.
- **Performance Monitoring**: Utilize Next.js built-in analytics or external tools (e.g., Vercel Analytics, Lighthouse) to monitor performance.
- **Structured Logging**: Implement structured logging to capture critical events and facilitate debugging.

---

## 🗺️ Map Integration

- **OpenStreetMap**: Integrate OpenStreetMap for displaying real-time and historical locations of robots.
- **Leaflet.js**: Consider using Leaflet.js for interactive map features and overlays.
- **Optimized Rendering**: Ensure map rendering is optimized for performance, especially when handling numerous markers or paths.

---

## 🤖 OpenAI API Integration

- **Secure API Calls**: Ensure API keys are stored securely and never exposed to the client.
- **Role-Based Access**: Restrict access to the OpenAI-powered command interface to Admin users only.
- **Input Validation**: Validate and sanitize user inputs before passing them to the OpenAI API.

---

## 📝 Audit Logging

- **User Actions**: Log significant user actions (logins, configuration changes, command executions) for accountability.
- **Secure Log Storage**: Ensure logs are stored securely and are tamper-proof.
- **Log Rotation**: Implement log rotation policies to manage disk space effectively.
- **Monitoring**: Use monitoring tools to analyze logs and detect anomalies.

---

## 🎨 User Interface and Experience

- **Consistent Design**: Maintain a consistent UI design using blue as the primary color.
- **Accessibility**: Ensure the interface meets accessibility standards (e.g., WCAG) to accommodate all users.
- **Localization**: Support multiple languages to cater to a global user base.
- **Interactive Feedback**: Provide immediate visual feedback for user actions and interactions.

---

## 📄 Documentation

- **Code Documentation**: Use JSDoc or TypeScript doc comments to document functions and components.
- **API Documentation**: Maintain comprehensive API documentation using tools like Swagger or OpenAPI.
- **Developer Guides**: Create guides on project setup, development workflow, and deployment procedures for new developers.
- **User Documentation**: Provide user manuals and FAQs to assist users in navigating the dashboard.
---

## 📌 Final Notes

- **Stay Updated**: Keep dependencies and Next.js versions updated to leverage the latest features and security patches.
- **Continuous Improvement**: Regularly refactor and improve code based on feedback, performance metrics, and evolving requirements.
- **Feedback Loop**: Ensure there’s a system for collecting user feedback and addressing issues promptly.

---
