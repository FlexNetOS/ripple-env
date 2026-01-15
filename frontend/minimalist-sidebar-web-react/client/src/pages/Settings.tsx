import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Switch } from "@/components/ui/switch";
import { Separator } from "@/components/ui/separator";
import { useAuth } from "@/_core/hooks/useAuth";
import { trpc } from "@/lib/trpc";
import { useState, useEffect, useRef } from "react";
import {
  User,
  Bell,
  Shield,
  Loader2,
  Save,
  Mail,
  Smartphone,
  Camera,
  Upload,
} from "lucide-react";
import { toast } from "sonner";

export default function Settings() {
  const { user } = useAuth();
  const utils = trpc.useUtils();
  const fileInputRef = useRef<HTMLInputElement>(null);

  const [name, setName] = useState(user?.name || "");
  const [email, setEmail] = useState(user?.email || "");
  const [avatarPreview, setAvatarPreview] = useState<string | null>(null);
  const [isUploadingAvatar, setIsUploadingAvatar] = useState(false);

  const { data: preferences, isLoading: loadingPrefs } =
    trpc.notifications.getPreferences.useQuery();

  const [emailNotifications, setEmailNotifications] = useState(true);
  const [pushNotifications, setPushNotifications] = useState(true);

  useEffect(() => {
    if (preferences) {
      setEmailNotifications(preferences.emailEnabled);
      setPushNotifications(preferences.pushEnabled);
    }
  }, [preferences]);

  useEffect(() => {
    if (user) {
      setName(user.name || "");
      setEmail(user.email || "");
      if (user.avatarUrl) {
        setAvatarPreview(user.avatarUrl);
      }
    }
  }, [user]);

  const updateProfile = trpc.user.updateProfile.useMutation({
    onSuccess: () => {
      utils.auth.me.invalidate();
      toast.success("Profile updated successfully");
    },
    onError: (error) => {
      toast.error("Failed to update profile: " + error.message);
    },
  });

  const uploadAvatar = trpc.user.uploadAvatar.useMutation({
    onSuccess: (data) => {
      utils.auth.me.invalidate();
      setAvatarPreview(data.avatarUrl);
      toast.success("Avatar uploaded successfully");
    },
    onError: (error) => {
      toast.error("Failed to upload avatar: " + error.message);
    },
  });

  const updatePreferences = trpc.notifications.updatePreferences.useMutation({
    onSuccess: () => {
      utils.notifications.getPreferences.invalidate();
      toast.success("Notification preferences updated");
    },
    onError: (error: { message: string }) => {
      toast.error("Failed to update preferences: " + error.message);
    },
  });

  const handleSaveProfile = () => {
    updateProfile.mutate({ name, email });
  };

  const handleToggleEmailNotifications = (checked: boolean) => {
    setEmailNotifications(checked);
    updatePreferences.mutate({ emailEnabled: checked });
  };

  const handleTogglePushNotifications = (checked: boolean) => {
    setPushNotifications(checked);
    updatePreferences.mutate({ pushEnabled: checked });
  };

  const handleAvatarClick = () => {
    fileInputRef.current?.click();
  };

  const handleFileChange = async (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;

    // Validate file type
    if (!file.type.startsWith('image/')) {
      toast.error('Please select an image file');
      return;
    }

    // Validate file size (max 5MB)
    if (file.size > 5 * 1024 * 1024) {
      toast.error('Image must be less than 5MB');
      return;
    }

    setIsUploadingAvatar(true);

    try {
      // Convert to base64
      const reader = new FileReader();
      reader.onloadend = () => {
        const base64 = reader.result as string;
        // Show preview immediately
        setAvatarPreview(base64);
        
        // Upload to server
        uploadAvatar.mutate({
          base64Data: base64,
          fileName: file.name,
          mimeType: file.type,
        });
      };
      reader.readAsDataURL(file);
    } catch {
      toast.error('Failed to process image');
    } finally {
      setIsUploadingAvatar(false);
    }

    // Reset input
    e.target.value = '';
  };

  const getInitials = () => {
    if (name) {
      return name.split(' ').map(n => n[0]).join('').toUpperCase().slice(0, 2);
    }
    if (email) {
      return email[0].toUpperCase();
    }
    return 'U';
  };

  return (
    <div className="p-6 space-y-8 max-w-2xl">
      {/* Header */}
      <div>
        <h1 className="text-3xl font-bold tracking-tight text-white">Settings</h1>
        <p className="text-[#9BA1A6] mt-1">
          Manage your account settings and preferences
        </p>
      </div>

      {/* Profile Settings */}
      <Card className="bg-[#1A1A1A] border-[#2A2A2A]">
        <CardHeader>
          <div className="flex items-center gap-3">
            <div className="flex h-10 w-10 items-center justify-center rounded-lg bg-[#00D4FF]/10">
              <User className="h-5 w-5 text-[#00D4FF]" />
            </div>
            <div>
              <CardTitle className="text-white">Profile</CardTitle>
              <CardDescription className="text-[#6B7280]">Your personal information</CardDescription>
            </div>
          </div>
        </CardHeader>
        <CardContent className="space-y-6">
          {/* Avatar Upload */}
          <div className="flex items-center gap-6">
            <div className="relative group">
              <input
                ref={fileInputRef}
                type="file"
                accept="image/*"
                onChange={handleFileChange}
                className="hidden"
              />
              <button
                onClick={handleAvatarClick}
                disabled={isUploadingAvatar || uploadAvatar.isPending}
                className="relative w-24 h-24 rounded-full overflow-hidden border-2 border-[#3A3A3A] hover:border-[#00D4FF] transition-colors group"
              >
                {avatarPreview ? (
                  <img
                    src={avatarPreview}
                    alt="Avatar"
                    className="w-full h-full object-cover"
                  />
                ) : (
                  <div className="w-full h-full bg-gradient-to-br from-[#00D4FF] to-[#9B7BFF] flex items-center justify-center text-white text-2xl font-bold">
                    {getInitials()}
                  </div>
                )}
                {/* Overlay */}
                <div className="absolute inset-0 bg-black/60 flex items-center justify-center opacity-0 group-hover:opacity-100 transition-opacity">
                  {isUploadingAvatar || uploadAvatar.isPending ? (
                    <Loader2 className="h-6 w-6 text-white animate-spin" />
                  ) : (
                    <Camera className="h-6 w-6 text-white" />
                  )}
                </div>
              </button>
              {/* Upload badge */}
              <div className="absolute -bottom-1 -right-1 w-8 h-8 rounded-full bg-[#00D4FF] flex items-center justify-center border-2 border-[#1A1A1A]">
                <Upload className="h-4 w-4 text-white" />
              </div>
            </div>
            <div className="flex-1">
              <p className="text-sm font-medium text-white">Profile Photo</p>
              <p className="text-xs text-[#6B7280] mt-1">
                Click to upload a new avatar. Max size 5MB.
              </p>
              <p className="text-xs text-[#6B7280]">
                Recommended: Square image, at least 200x200px
              </p>
            </div>
          </div>

          <Separator className="bg-[#2A2A2A]" />

          <div className="space-y-2">
            <Label htmlFor="name" className="text-[#9BA1A6]">Display Name</Label>
            <Input
              id="name"
              value={name}
              onChange={(e) => setName(e.target.value)}
              placeholder="Enter your name"
              className="bg-[#252525] border-[#3A3A3A] text-white placeholder:text-[#6B7280]"
            />
          </div>
          <div className="space-y-2">
            <Label htmlFor="email" className="text-[#9BA1A6]">Email Address</Label>
            <Input
              id="email"
              type="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              placeholder="Enter your email"
              className="bg-[#252525] border-[#3A3A3A] text-white placeholder:text-[#6B7280]"
            />
          </div>
        </CardContent>
        <CardFooter>
          <Button
            onClick={handleSaveProfile}
            disabled={updateProfile.isPending}
            className="bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] hover:opacity-90 text-white"
          >
            {updateProfile.isPending ? (
              <Loader2 className="h-4 w-4 mr-2 animate-spin" />
            ) : (
              <Save className="h-4 w-4 mr-2" />
            )}
            Save Changes
          </Button>
        </CardFooter>
      </Card>

      {/* Notification Settings */}
      <Card className="bg-[#1A1A1A] border-[#2A2A2A]">
        <CardHeader>
          <div className="flex items-center gap-3">
            <div className="flex h-10 w-10 items-center justify-center rounded-lg bg-[#9B7BFF]/10">
              <Bell className="h-5 w-5 text-[#9B7BFF]" />
            </div>
            <div>
              <CardTitle className="text-white">Notifications</CardTitle>
              <CardDescription className="text-[#6B7280]">Configure how you receive notifications</CardDescription>
            </div>
          </div>
        </CardHeader>
        <CardContent className="space-y-6">
          {loadingPrefs ? (
            <div className="flex items-center justify-center py-4">
              <Loader2 className="h-6 w-6 animate-spin text-[#00D4FF]" />
            </div>
          ) : (
            <>
              <div className="flex items-center justify-between">
                <div className="flex items-center gap-3">
                  <Mail className="h-5 w-5 text-[#9BA1A6]" />
                  <div>
                    <p className="font-medium text-white">Email Notifications</p>
                    <p className="text-sm text-[#6B7280]">
                      Receive notifications via email
                    </p>
                  </div>
                </div>
                <Switch
                  checked={emailNotifications}
                  onCheckedChange={handleToggleEmailNotifications}
                  disabled={updatePreferences.isPending}
                />
              </div>
              <Separator className="bg-[#2A2A2A]" />
              <div className="flex items-center justify-between">
                <div className="flex items-center gap-3">
                  <Smartphone className="h-5 w-5 text-[#9BA1A6]" />
                  <div>
                    <p className="font-medium text-white">Push Notifications</p>
                    <p className="text-sm text-[#6B7280]">
                      Receive in-app push notifications
                    </p>
                  </div>
                </div>
                <Switch
                  checked={pushNotifications}
                  onCheckedChange={handleTogglePushNotifications}
                  disabled={updatePreferences.isPending}
                />
              </div>
            </>
          )}
        </CardContent>
      </Card>

      {/* Security Settings */}
      <Card className="bg-[#1A1A1A] border-[#2A2A2A]">
        <CardHeader>
          <div className="flex items-center gap-3">
            <div className="flex h-10 w-10 items-center justify-center rounded-lg bg-[#00E676]/10">
              <Shield className="h-5 w-5 text-[#00E676]" />
            </div>
            <div>
              <CardTitle className="text-white">Security</CardTitle>
              <CardDescription className="text-[#6B7280]">Account security information</CardDescription>
            </div>
          </div>
        </CardHeader>
        <CardContent className="space-y-4">
          <div className="flex items-center justify-between py-2">
            <div>
              <p className="font-medium text-white">Authentication Method</p>
              <p className="text-sm text-[#6B7280]">
                {user?.loginMethod || "Manus OAuth"}
              </p>
            </div>
          </div>
          <Separator className="bg-[#2A2A2A]" />
          <div className="flex items-center justify-between py-2">
            <div>
              <p className="font-medium text-white">Account Role</p>
              <p className="text-sm text-[#6B7280] capitalize">
                {user?.role || "User"}
              </p>
            </div>
          </div>
          <Separator className="bg-[#2A2A2A]" />
          <div className="flex items-center justify-between py-2">
            <div>
              <p className="font-medium text-white">Last Sign In</p>
              <p className="text-sm text-[#6B7280]">
                {user?.lastSignedIn
                  ? new Date(user.lastSignedIn).toLocaleString()
                  : "Unknown"}
              </p>
            </div>
          </div>
        </CardContent>
      </Card>
    </div>
  );
}
