"use client";

import { useRouter } from "next/navigation";
import { useEffect, useState } from "react";
import CommandInterface from "../../../components/ai/CommandInterface";
import { useAuth } from "../../../hooks/useAuth";

/**
 * AI Command Page
 * 
 * This page provides administrators with an interface for sending natural language
 * commands to robots via the OpenAI API as specified in FR015.
 * 
 * Access to this page is restricted to users with Admin roles only.
 */
export default function AICommandPage() {
  const router = useRouter();
  const { user, isLoading } = useAuth();
  const [isAuthorized, setIsAuthorized] = useState(false);

  // Check if the user has admin privileges
  useEffect(() => {
    if (!isLoading) {
      if (!user) {
        // Redirect to login if not authenticated
        router.push("/auth/login?redirect=/dashboard/ai");
      } else if (user.role !== "admin") {
        // Check if the user is an admin
        setIsAuthorized(false);
      } else {
        setIsAuthorized(true);
      }
    }
  }, [user, isLoading, router]);

  // Show loading state while checking authorization
  if (isLoading) {
    return (
      <div className="flex justify-center items-center h-64">
        <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-700"></div>
      </div>
    );
  }

  // Show unauthorized message for non-admin users
  if (!isAuthorized) {
    return (
      <div className="p-6">
        <div className="bg-red-50 dark:bg-red-900/20 p-4 rounded-lg text-center">
          <h2 className="text-xl font-bold text-red-800 dark:text-red-200 mb-2">
            Access Denied
          </h2>
          <p className="text-red-600 dark:text-red-300">
            You do not have permission to access this feature.
            This feature is only available to administrators.
          </p>
        </div>
      </div>
    );
  }

  return (
    <div className="p-6">
      <h1 className="text-2xl font-bold mb-6">AI Robot Command Interface</h1>
      
      <div className="mb-6">
        <p className="text-gray-700 dark:text-gray-300">
          Use natural language to control and manage robots in the WiseVision ecosystem.
          Simply type your command in plain English, and the AI will convert it into 
          specific robot instructions.
        </p>
      </div>
      
      <CommandInterface />
      
      <div className="mt-8 bg-blue-50 dark:bg-blue-900/20 p-4 rounded-lg">
        <h3 className="text-lg font-semibold mb-2">Tips for effective commands:</h3>
        <ul className="list-disc list-inside space-y-1 text-sm">
          <li>Specify the target robot in your command (e.g., "robot_1" or "delivery_bot")</li>
          <li>Be clear about the action you want the robot to perform</li>
          <li>Include relevant parameters like speed, direction, or location</li>
          <li>Examples:
            <ul className="list-disc list-inside ml-4 text-gray-600 dark:text-gray-400">
              <li>"Move robot_1 forward at medium speed"</li>
              <li>"Tell delivery_bot to navigate to the kitchen"</li>
              <li>"Make survey_drone scan the warehouse area"</li>
              <li>"Stop all robots immediately"</li>
            </ul>
          </li>
        </ul>
      </div>
    </div>
  );
}