# Authentication System Documentation

## Overview

This application supports **three authentication modes** to work in different environments:

1. **Demo Mode** - Bypass authentication with a mock user (for local development/preview)
2. **Manus OAuth** - Use Manus authentication (when running in Manus environment)
3. **Google OAuth** - Use Google authentication (works anywhere, requires Google Cloud setup)

## Quick Start

### Demo Mode (Default)

Perfect for local development and preview. No configuration needed!

```bash
# .env file
AUTH_MODE=demo
```

The app will automatically use a mock user:
- Name: Demo User
- Email: demo@ripple.example
- ID: demo-user-123

### Manus OAuth Mode

Use this when running inside the Manus environment.

```bash
# .env file
AUTH_MODE=manus
OAUTH_SERVER_URL=https://api.manus.im
VITE_OAUTH_PORTAL_URL=https://oauth.manus.im
VITE_APP_ID=your-app-id
JWT_SECRET=your-secret-key
```

**How it works:**
1. User clicks "Sign in"
2. Redirects to Manus OAuth portal
3. User authenticates with Manus account
4. Redirects back with session token
5. App creates session cookie

### Google OAuth Mode

Use this for standalone deployment outside Manus.

```bash
# .env file
AUTH_MODE=google
GOOGLE_CLIENT_ID=your-google-client-id.apps.googleusercontent.com
GOOGLE_CLIENT_SECRET=your-google-client-secret
JWT_SECRET=your-secret-key
```

**Setup Instructions:**

1. Go to [Google Cloud Console](https://console.cloud.google.com/)
2. Create a new project or select existing
3. Enable Google+ API
4. Create OAuth 2.0 credentials
5. Add authorized redirect URI: `http://localhost:3000/api/oauth/google/callback`
6. Copy Client ID and Client Secret to `.env`

## Environment Variables Reference

| Variable | Required For | Description |
|----------|-------------|-------------|
| `AUTH_MODE` | All | Authentication mode: `demo`, `manus`, or `google` |
| `JWT_SECRET` | manus, google | Secret key for signing session tokens |
| `OAUTH_SERVER_URL` | manus | Manus OAuth API endpoint |
| `VITE_OAUTH_PORTAL_URL` | manus | Manus OAuth portal URL (for frontend) |
| `VITE_APP_ID` | manus | Your Manus app ID |
| `GOOGLE_CLIENT_ID` | google | Google OAuth client ID |
| `GOOGLE_CLIENT_SECRET` | google | Google OAuth client secret |

## Architecture

### Authentication Flow

```
┌─────────────┐
│   Browser   │
└──────┬──────┘
       │ 1. Click "Sign in"
       ▼
┌─────────────────┐
│  Frontend       │
│  (React)        │
└──────┬──────────┘
       │ 2. Redirect to OAuth
       ▼
┌─────────────────┐
│  OAuth Provider │
│  (Manus/Google) │
└──────┬──────────┘
       │ 3. Authenticate
       ▼
┌─────────────────┐
│  Backend        │
│  (Express)      │
└──────┬──────────┘
       │ 4. Exchange code for token
       │ 5. Get user info
       │ 6. Create session
       ▼
┌─────────────────┐
│  Database       │
│  (MySQL)        │
└─────────────────┘
```

### Session Management

- Sessions are stored as **JWT tokens** in HTTP-only cookies
- Cookie name: `manus-session` (defined in `@shared/const`)
- Default expiration: 1 year
- Automatic refresh on each request

### Demo Mode Implementation

When `AUTH_MODE=demo`:
- `context.ts` returns a mock user object
- No OAuth calls are made
- No database queries for user
- All protected routes work immediately

## File Structure

```
server/
├── _core/
│   ├── context.ts       # Creates tRPC context with user
│   ├── env.ts           # Environment variables
│   ├── oauth.ts         # OAuth callback handlers
│   ├── sdk.ts           # OAuth SDK and session management
│   └── trpc.ts          # tRPC setup with auth middleware
├── db.ts                # Database operations
└── routers.ts           # API routes

client/
├── src/
│   ├── _core/
│   │   └── hooks/
│   │       └── useAuth.ts    # Auth hook for components
│   ├── const.ts              # Login URL generator
│   └── pages/
│       └── Home.tsx          # Sign-in page
```

## Adding More OAuth Providers

To add a new provider (e.g., GitHub, Microsoft):

1. **Update `.env`:**
   ```bash
   AUTH_MODE=github
   GITHUB_CLIENT_ID=...
   GITHUB_CLIENT_SECRET=...
   ```

2. **Update `env.ts`:**
   ```typescript
   export const ENV = {
     authMode: (process.env.AUTH_MODE ?? "demo") as "demo" | "manus" | "google" | "github",
     githubClientId: process.env.GITHUB_CLIENT_ID ?? "",
     githubClientSecret: process.env.GITHUB_CLIENT_SECRET ?? "",
     // ...
   };
   ```

3. **Add OAuth handler in `oauth.ts`:**
   ```typescript
   app.get("/api/oauth/github/callback", async (req, res) => {
     // Implement GitHub OAuth flow
   });
   ```

4. **Update `context.ts`:**
   ```typescript
   if (ENV.authMode === 'github') {
     // Implement GitHub authentication
   }
   ```

## Security Best Practices

### Production Deployment

1. **Change JWT_SECRET:**
   ```bash
   # Generate a secure random secret
   openssl rand -base64 32
   ```

2. **Use HTTPS:**
   - All OAuth providers require HTTPS in production
   - Update redirect URIs to use `https://`

3. **Set Secure Cookies:**
   - Automatically enabled in production (`NODE_ENV=production`)
   - Cookies are HTTP-only and secure

4. **Whitelist Domains:**
   - Update `vite.config.ts` allowed hosts
   - Configure OAuth redirect URIs

### Database

- User data is stored in MySQL/TiDB
- Schema defined in `drizzle/schema.ts`
- Automatic user creation on first login
- Last sign-in timestamp tracked

## Troubleshooting

### "Sign in" button not working

**Demo Mode:**
- Check `.env` has `AUTH_MODE=demo`
- Restart server after changing `.env`
- Check browser console for errors

**Manus OAuth:**
- Verify `OAUTH_SERVER_URL` is correct
- Check `VITE_APP_ID` matches your Manus app
- Ensure redirect URI is whitelisted in Manus

**Google OAuth:**
- Verify credentials in Google Cloud Console
- Check redirect URI matches exactly
- Enable Google+ API

### Session expires immediately

- Check `JWT_SECRET` is set
- Verify cookie is being set (check browser DevTools)
- Check server logs for JWT errors

### Database errors

- Ensure MySQL is running
- Check `DATABASE_URL` is correct
- Run migrations: `pnpm db:push`

## Development Tips

### Testing Different Auth Modes

```bash
# Terminal 1: Demo mode
AUTH_MODE=demo pnpm dev

# Terminal 2: Manus mode
AUTH_MODE=manus pnpm dev

# Terminal 3: Google mode
AUTH_MODE=google pnpm dev
```

### Debugging Auth Flow

Add logging in `context.ts`:
```typescript
console.log('[Auth] Mode:', ENV.authMode);
console.log('[Auth] User:', user);
```

### Bypassing Auth for Specific Routes

Make routes public by using `publicProcedure` instead of `protectedProcedure`:

```typescript
// In routers.ts
myRoute: publicProcedure.query(async () => {
  // No authentication required
});
```

## Support

For issues or questions:
- Check server logs for error messages
- Review browser console for client-side errors
- Verify environment variables are loaded correctly
- Test with demo mode first to isolate OAuth issues
