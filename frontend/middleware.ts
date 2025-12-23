// frontend/middleware.ts
import { auth } from "./auth"; // Import the 'auth' object from your auth.ts file

export default auth((req) => {
  // Example: Redirect unauthenticated users from a protected route
  // If you want to protect all routes except login, you can do this:
  if (!req.auth && req.nextUrl.pathname !== "/login" && !req.nextUrl.pathname.startsWith("/api/auth")) {
    const newUrl = new URL("/login", req.nextUrl.origin);
    return Response.redirect(newUrl);
  }
});

export const config = {
  // Apply middleware to all routes except API routes, static files, and _next
  // It's important to exclude /api/auth here, as NextAuth.js needs to handle its own routes
  matcher: ["/((?!api/auth|_next/static|_next/image|favicon.ico).*)"],
};