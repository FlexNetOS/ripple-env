import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { trpc } from "@/lib/trpc";
import { useState, useRef } from "react";
import {
  FileUp,
  File,
  Image,
  FileText,
  FileVideo,
  FileAudio,
  Trash2,
  Download,
  Loader2,
  Upload,
  Search,
  MoreVertical,
  ExternalLink,
} from "lucide-react";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu";
import {
  AlertDialog,
  AlertDialogAction,
  AlertDialogCancel,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
} from "@/components/ui/alert-dialog";
import { toast } from "sonner";

function getFileIcon(mimeType: string | null) {
  if (!mimeType) return File;
  if (mimeType.startsWith("image/")) return Image;
  if (mimeType.startsWith("video/")) return FileVideo;
  if (mimeType.startsWith("audio/")) return FileAudio;
  if (mimeType.includes("pdf") || mimeType.includes("document")) return FileText;
  return File;
}

function formatFileSize(bytes: number | null) {
  if (!bytes) return "Unknown size";
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
  return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
}

export default function Files() {
  const [searchQuery, setSearchQuery] = useState("");
  const [deleteDialogOpen, setDeleteDialogOpen] = useState(false);
  const [fileToDelete, setFileToDelete] = useState<number | null>(null);
  const [uploading, setUploading] = useState(false);
  const fileInputRef = useRef<HTMLInputElement>(null);
  const utils = trpc.useUtils();

  const { data: files, isLoading } = trpc.files.list.useQuery();

  const uploadFile = trpc.files.upload.useMutation({
    onSuccess: () => {
      utils.files.list.invalidate();
      toast.success("File uploaded successfully");
      setUploading(false);
    },
    onError: (error) => {
      toast.error("Upload failed: " + error.message);
      setUploading(false);
    },
  });

  const deleteFile = trpc.files.delete.useMutation({
    onSuccess: () => {
      utils.files.list.invalidate();
      toast.success("File deleted");
      setDeleteDialogOpen(false);
      setFileToDelete(null);
    },
    onError: (error) => {
      toast.error("Delete failed: " + error.message);
    },
  });

  const handleFileSelect = async (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;

    // Check file size (max 10MB)
    if (file.size > 10 * 1024 * 1024) {
      toast.error("File size must be less than 10MB");
      return;
    }

    setUploading(true);

    // Convert to base64
    const reader = new FileReader();
    reader.onload = () => {
      const base64 = (reader.result as string).split(",")[1];
      uploadFile.mutate({
        fileName: file.name,
        fileData: base64,
        mimeType: file.type || undefined,
      });
    };
    reader.onerror = () => {
      toast.error("Failed to read file");
      setUploading(false);
    };
    reader.readAsDataURL(file);

    // Reset input
    if (fileInputRef.current) {
      fileInputRef.current.value = "";
    }
  };

  const handleDeleteClick = (id: number) => {
    setFileToDelete(id);
    setDeleteDialogOpen(true);
  };

  const confirmDelete = () => {
    if (fileToDelete) {
      deleteFile.mutate({ id: fileToDelete });
    }
  };

  const filteredFiles = files?.filter((file) =>
    file.fileName.toLowerCase().includes(searchQuery.toLowerCase())
  );

  return (
    <div className="p-6 space-y-6">
      {/* Header */}
      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4">
        <div>
          <h1 className="text-3xl font-bold tracking-tight text-white">Files</h1>
          <p className="text-[#9BA1A6] mt-1">
            Upload and manage your files securely
          </p>
        </div>
        <div>
          <input
            type="file"
            ref={fileInputRef}
            onChange={handleFileSelect}
            className="hidden"
          />
          <Button
            onClick={() => fileInputRef.current?.click()}
            disabled={uploading}
            className="bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] hover:opacity-90 text-white"
          >
            {uploading ? (
              <Loader2 className="h-4 w-4 mr-2 animate-spin" />
            ) : (
              <Upload className="h-4 w-4 mr-2" />
            )}
            Upload File
          </Button>
        </div>
      </div>

      {/* Search */}
      <div className="relative max-w-md">
        <Search className="absolute left-3 top-1/2 -translate-y-1/2 h-4 w-4 text-[#6B7280]" />
        <Input
          placeholder="Search files..."
          value={searchQuery}
          onChange={(e) => setSearchQuery(e.target.value)}
          className="pl-10 bg-[#1A1A1A] border-[#2A2A2A] text-white placeholder:text-[#6B7280]"
        />
      </div>

      {/* Files Grid */}
      {isLoading ? (
        <div className="flex items-center justify-center py-12">
          <Loader2 className="h-8 w-8 animate-spin text-[#00D4FF]" />
        </div>
      ) : filteredFiles && filteredFiles.length > 0 ? (
        <div className="grid gap-4 sm:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4">
          {filteredFiles.map((file) => {
            const FileIcon = getFileIcon(file.mimeType);
            return (
              <Card key={file.id} className="group bg-[#1A1A1A] border-[#2A2A2A]">
                <CardHeader className="pb-3">
                  <div className="flex items-start justify-between">
                    <div className="flex h-12 w-12 items-center justify-center rounded-lg bg-[#00D4FF]/10">
                      <FileIcon className="h-6 w-6 text-[#00D4FF]" />
                    </div>
                    <DropdownMenu>
                      <DropdownMenuTrigger asChild>
                        <Button
                          variant="ghost"
                          size="icon"
                          className="h-8 w-8 opacity-0 group-hover:opacity-100"
                        >
                          <MoreVertical className="h-4 w-4" />
                        </Button>
                      </DropdownMenuTrigger>
                      <DropdownMenuContent align="end">
                        <DropdownMenuItem asChild>
                          <a href={file.url} target="_blank" rel="noopener noreferrer">
                            <ExternalLink className="h-4 w-4 mr-2" />
                            Open
                          </a>
                        </DropdownMenuItem>
                        <DropdownMenuItem asChild>
                          <a href={file.url} download={file.fileName}>
                            <Download className="h-4 w-4 mr-2" />
                            Download
                          </a>
                        </DropdownMenuItem>
                        <DropdownMenuItem
                          className="text-destructive"
                          onClick={() => handleDeleteClick(file.id)}
                        >
                          <Trash2 className="h-4 w-4 mr-2" />
                          Delete
                        </DropdownMenuItem>
                      </DropdownMenuContent>
                    </DropdownMenu>
                  </div>
                </CardHeader>
                <CardContent>
                  <CardTitle className="text-sm font-medium truncate mb-1 text-white">
                    {file.fileName}
                  </CardTitle>
                  <CardDescription className="text-xs text-[#6B7280]">
                    {formatFileSize(file.fileSize)} • {new Date(file.createdAt).toLocaleDateString()}
                  </CardDescription>
                </CardContent>
              </Card>
            );
          })}
        </div>
      ) : (
        <Card className="py-12 bg-[#1A1A1A] border-[#2A2A2A]">
          <div className="text-center">
            <FileUp className="h-16 w-16 mx-auto mb-4 text-[#00D4FF] opacity-50" />
            <h3 className="text-lg font-medium mb-2 text-white">No files yet</h3>
            <p className="text-[#6B7280] mb-6">
              Upload your first file to get started
            </p>
            <Button
              onClick={() => fileInputRef.current?.click()}
              disabled={uploading}
              className="bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] hover:opacity-90 text-white"
            >
              <Upload className="h-4 w-4 mr-2" />
              Upload File
            </Button>
          </div>
        </Card>
      )}

      {/* Delete Confirmation Dialog */}
      <AlertDialog open={deleteDialogOpen} onOpenChange={setDeleteDialogOpen}>
        <AlertDialogContent className="bg-[#1A1A1A] border-[#2A2A2A]">
          <AlertDialogHeader>
            <AlertDialogTitle className="text-white">Delete File</AlertDialogTitle>
            <AlertDialogDescription className="text-[#9BA1A6]">
              Are you sure you want to delete this file? This action cannot be undone.
            </AlertDialogDescription>
          </AlertDialogHeader>
          <AlertDialogFooter>
            <AlertDialogCancel className="bg-[#252525] border-[#3A3A3A] text-white hover:bg-[#3A3A3A]">Cancel</AlertDialogCancel>
            <AlertDialogAction
              onClick={confirmDelete}
              className="bg-red-500 text-white hover:bg-red-600"
            >
              Delete
            </AlertDialogAction>
          </AlertDialogFooter>
        </AlertDialogContent>
      </AlertDialog>
    </div>
  );
}
