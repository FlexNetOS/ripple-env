import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { ScrollArea } from "@/components/ui/scroll-area";
import { trpc } from "@/lib/trpc";
import { useLocation } from "wouter";
import { useState, useEffect, useRef } from "react";
import { Streamdown } from "streamdown";
import {
  MessageSquare,
  Plus,
  Send,
  Trash2,
  Loader2,
  Sparkles,
  User,
  MoreVertical,
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

export default function Chat() {
  const [location, setLocation] = useLocation();
  const searchParams = new URLSearchParams(location.split("?")[1] || "");
  const conversationId = searchParams.get("id");

  const [message, setMessage] = useState("");
  const [deleteDialogOpen, setDeleteDialogOpen] = useState(false);
  const [conversationToDelete, setConversationToDelete] = useState<number | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const utils = trpc.useUtils();

  const { data: conversations, isLoading: loadingConversations } =
    trpc.chat.listConversations.useQuery();

  const { data: currentConversation, isLoading: loadingConversation } =
    trpc.chat.getConversation.useQuery(
      { id: parseInt(conversationId || "0") },
      { enabled: !!conversationId }
    );

  const createConversation = trpc.chat.createConversation.useMutation({
    onSuccess: (data) => {
      utils.chat.listConversations.invalidate();
      setLocation(`/chat?id=${data.id}`);
    },
  });

  const sendMessage = trpc.chat.sendMessage.useMutation({
    onSuccess: () => {
      utils.chat.getConversation.invalidate({ id: parseInt(conversationId || "0") });
      utils.chat.listConversations.invalidate();
      setMessage("");
    },
    onError: (error) => {
      toast.error("Failed to send message: " + error.message);
    },
  });

  const deleteConversation = trpc.chat.deleteConversation.useMutation({
    onSuccess: () => {
      utils.chat.listConversations.invalidate();
      if (conversationId && parseInt(conversationId) === conversationToDelete) {
        setLocation("/chat");
      }
      toast.success("Conversation deleted");
      setDeleteDialogOpen(false);
      setConversationToDelete(null);
    },
    onError: (error) => {
      toast.error("Failed to delete: " + error.message);
    },
  });

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [currentConversation?.messages]);

  const handleSend = () => {
    if (!message.trim() || !conversationId) return;
    sendMessage.mutate({
      conversationId: parseInt(conversationId),
      content: message.trim(),
    });
  };

  const handleNewChat = () => {
    createConversation.mutate({});
  };

  const handleDeleteClick = (id: number, e: React.MouseEvent) => {
    e.stopPropagation();
    setConversationToDelete(id);
    setDeleteDialogOpen(true);
  };

  const confirmDelete = () => {
    if (conversationToDelete) {
      deleteConversation.mutate({ id: conversationToDelete });
    }
  };

  return (
    <div className="p-6 h-full">
      <div className="flex h-[calc(100vh-6rem)] gap-4">
        {/* Sidebar - Conversation List */}
        <Card className="w-72 flex-shrink-0 flex flex-col bg-[#1A1A1A] border-[#2A2A2A]">
          <div className="p-4 border-b border-[#2A2A2A]">
            <Button 
              onClick={handleNewChat} 
              className="w-full bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] hover:opacity-90 text-white" 
              disabled={createConversation.isPending}
            >
              {createConversation.isPending ? (
                <Loader2 className="h-4 w-4 mr-2 animate-spin" />
              ) : (
                <Plus className="h-4 w-4 mr-2" />
              )}
              New Chat
            </Button>
          </div>
          <ScrollArea className="flex-1">
            <div className="p-2 space-y-1">
              {loadingConversations ? (
                <div className="flex items-center justify-center py-8">
                  <Loader2 className="h-6 w-6 animate-spin text-[#9BA1A6]" />
                </div>
              ) : conversations && conversations.length > 0 ? (
                conversations.map((conv) => (
                  <div
                    key={conv.id}
                    className={`group flex items-center gap-2 rounded-lg p-3 cursor-pointer transition-colors ${
                      conversationId && parseInt(conversationId) === conv.id
                        ? "bg-[#00D4FF]/10 text-[#00D4FF]"
                        : "hover:bg-[#252525] text-white"
                    }`}
                    onClick={() => setLocation(`/chat?id=${conv.id}`)}
                  >
                    <MessageSquare className="h-4 w-4 flex-shrink-0" />
                    <span className="flex-1 truncate text-sm">{conv.title}</span>
                    <DropdownMenu>
                      <DropdownMenuTrigger asChild>
                        <Button
                          variant="ghost"
                          size="icon"
                          className="h-6 w-6 opacity-0 group-hover:opacity-100"
                          onClick={(e) => e.stopPropagation()}
                        >
                          <MoreVertical className="h-3 w-3" />
                        </Button>
                      </DropdownMenuTrigger>
                      <DropdownMenuContent align="end">
                        <DropdownMenuItem
                          className="text-destructive"
                          onClick={(e) => handleDeleteClick(conv.id, e as unknown as React.MouseEvent)}
                        >
                          <Trash2 className="h-4 w-4 mr-2" />
                          Delete
                        </DropdownMenuItem>
                      </DropdownMenuContent>
                    </DropdownMenu>
                  </div>
                ))
              ) : (
                <div className="text-center py-8 text-[#6B7280] text-sm">
                  No conversations yet
                </div>
              )}
            </div>
          </ScrollArea>
        </Card>

        {/* Main Chat Area */}
        <Card className="flex-1 flex flex-col bg-[#1A1A1A] border-[#2A2A2A]">
          {conversationId ? (
            <>
              {/* Messages */}
              <ScrollArea className="flex-1 p-4">
                {loadingConversation ? (
                  <div className="flex items-center justify-center h-full">
                    <Loader2 className="h-8 w-8 animate-spin text-[#9BA1A6]" />
                  </div>
                ) : currentConversation?.messages && currentConversation.messages.length > 0 ? (
                  <div className="space-y-4 max-w-3xl mx-auto">
                    {currentConversation.messages.map((msg) => (
                      <div
                        key={msg.id}
                        className={`flex gap-3 ${
                          msg.role === "user" ? "justify-end" : "justify-start"
                        }`}
                      >
                        {msg.role === "assistant" && (
                          <div className="flex h-8 w-8 items-center justify-center rounded-full bg-[#00D4FF]/10 flex-shrink-0">
                            <Sparkles className="h-4 w-4 text-[#00D4FF]" />
                          </div>
                        )}
                        <div
                          className={`rounded-2xl px-4 py-3 max-w-[80%] ${
                            msg.role === "user"
                              ? "bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] text-white"
                              : "bg-[#252525] text-white"
                          }`}
                        >
                          {msg.role === "assistant" ? (
                            <Streamdown>{msg.content}</Streamdown>
                          ) : (
                            <p className="whitespace-pre-wrap">{msg.content}</p>
                          )}
                        </div>
                        {msg.role === "user" && (
                          <div className="flex h-8 w-8 items-center justify-center rounded-full bg-[#2A2A2A] flex-shrink-0">
                            <User className="h-4 w-4 text-white" />
                          </div>
                        )}
                      </div>
                    ))}
                    {sendMessage.isPending && (
                      <div className="flex gap-3 justify-start">
                        <div className="flex h-8 w-8 items-center justify-center rounded-full bg-[#00D4FF]/10 flex-shrink-0">
                          <Sparkles className="h-4 w-4 text-[#00D4FF]" />
                        </div>
                        <div className="rounded-2xl px-4 py-3 bg-[#252525]">
                          <Loader2 className="h-5 w-5 animate-spin text-[#00D4FF]" />
                        </div>
                      </div>
                    )}
                    <div ref={messagesEndRef} />
                  </div>
                ) : (
                  <div className="flex flex-col items-center justify-center h-full text-[#6B7280]">
                    <Sparkles className="h-12 w-12 mb-4 opacity-50 text-[#00D4FF]" />
                    <p className="text-lg font-medium text-white">Start a conversation</p>
                    <p className="text-sm">Send a message to begin chatting with AI</p>
                  </div>
                )}
              </ScrollArea>

              {/* Input */}
              <div className="p-4 border-t border-[#2A2A2A]">
                <form
                  onSubmit={(e) => {
                    e.preventDefault();
                    handleSend();
                  }}
                  className="flex gap-2 max-w-3xl mx-auto"
                >
                  <Input
                    value={message}
                    onChange={(e) => setMessage(e.target.value)}
                    placeholder="Type your message..."
                    disabled={sendMessage.isPending}
                    className="flex-1 bg-[#252525] border-[#3A3A3A] text-white placeholder:text-[#6B7280]"
                  />
                  <Button 
                    type="submit" 
                    disabled={!message.trim() || sendMessage.isPending}
                    className="bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] hover:opacity-90 text-white"
                  >
                    {sendMessage.isPending ? (
                      <Loader2 className="h-4 w-4 animate-spin" />
                    ) : (
                      <Send className="h-4 w-4" />
                    )}
                  </Button>
                </form>
              </div>
            </>
          ) : (
            <div className="flex flex-col items-center justify-center h-full text-[#6B7280]">
              <MessageSquare className="h-16 w-16 mb-4 opacity-50 text-[#00D4FF]" />
              <p className="text-xl font-medium mb-2 text-white">Welcome to AI Chat</p>
              <p className="text-sm mb-6">Select a conversation or start a new one</p>
              <Button 
                onClick={handleNewChat} 
                disabled={createConversation.isPending}
                className="bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] hover:opacity-90 text-white"
              >
                {createConversation.isPending ? (
                  <Loader2 className="h-4 w-4 mr-2 animate-spin" />
                ) : (
                  <Plus className="h-4 w-4 mr-2" />
                )}
                New Chat
              </Button>
            </div>
          )}
        </Card>
      </div>

      {/* Delete Confirmation Dialog */}
      <AlertDialog open={deleteDialogOpen} onOpenChange={setDeleteDialogOpen}>
        <AlertDialogContent className="bg-[#1A1A1A] border-[#2A2A2A]">
          <AlertDialogHeader>
            <AlertDialogTitle className="text-white">Delete Conversation</AlertDialogTitle>
            <AlertDialogDescription className="text-[#9BA1A6]">
              Are you sure you want to delete this conversation? This action cannot be undone.
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
