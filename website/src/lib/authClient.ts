import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.REACT_APP_API_URL || "http://localhost:3000",
  endpoints: {
    signUp: "/api/auth/signup",
    signIn: "/api/auth/signin",
    signOut: "/api/auth/signout",
    getSession: "/api/auth/session",
  },
});

export const useAuth = authClient.useSession;
export const signUp = authClient.signUp;
export const signIn = authClient.signIn;
export const signOut = authClient.signOut;
