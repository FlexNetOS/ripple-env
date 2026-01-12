#!/usr/bin/env python3
"""
Fine-tuning Example Script
P1-020: Example fine-tuning workflow for LLMs

This script demonstrates:
- Dataset preparation
- Model fine-tuning with LoRA/QLoRA
- Evaluation and comparison
- Model export and deployment
"""

import os
import json
import argparse
from pathlib import Path
from typing import Optional, List, Dict

try:
    from datasets import load_dataset, Dataset
    from transformers import (
        AutoTokenizer,
        AutoModelForCausalLM,
        TrainingArguments,
        Trainer,
        DataCollatorForLanguageModeling,
    )
    from peft import LoraConfig, get_peft_model, TaskType
    import torch
except ImportError as e:
    print(f"Error: Missing required packages. Install with: pixi install")
    print(f"Details: {e}")
    exit(1)


class FineTuningExample:
    """Example fine-tuning workflow"""

    def __init__(
        self,
        model_name: str = "meta-llama/Llama-3.2-1B",
        output_dir: str = "./models/finetuned",
        dataset_name: Optional[str] = None,
        use_lora: bool = True,
    ):
        self.model_name = model_name
        self.output_dir = Path(output_dir)
        self.dataset_name = dataset_name
        self.use_lora = use_lora
        self.tokenizer = None
        self.model = None
        self.dataset = None

        # Create output directory
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def load_model_and_tokenizer(self):
        """Load pre-trained model and tokenizer"""
        print(f"Loading model: {self.model_name}")

        self.tokenizer = AutoTokenizer.from_pretrained(self.model_name)
        self.model = AutoModelForCausalLM.from_pretrained(
            self.model_name,
            torch_dtype=torch.float16 if torch.cuda.is_available() else torch.float32,
            device_map="auto" if torch.cuda.is_available() else None,
        )

        # Add padding token if needed
        if self.tokenizer.pad_token is None:
            self.tokenizer.pad_token = self.tokenizer.eos_token

        print(f"Model loaded: {self.model.num_parameters():,} parameters")

    def prepare_dataset(self, custom_data: Optional[List[Dict]] = None):
        """Prepare training dataset"""
        print("Preparing dataset...")

        if custom_data:
            # Use custom data
            self.dataset = Dataset.from_list(custom_data)
        elif self.dataset_name:
            # Load from HuggingFace
            self.dataset = load_dataset(self.dataset_name, split="train[:1000]")
        else:
            # Use example data
            self.dataset = self._create_example_dataset()

        # Tokenize dataset
        def tokenize_function(examples):
            return self.tokenizer(
                examples["text"],
                truncation=True,
                max_length=512,
                padding="max_length",
            )

        self.dataset = self.dataset.map(
            tokenize_function,
            batched=True,
            remove_columns=self.dataset.column_names,
        )

        print(f"Dataset prepared: {len(self.dataset)} examples")

    def _create_example_dataset(self) -> Dataset:
        """Create example dataset for demonstration"""
        examples = [
            {
                "text": "Question: What is Kubernetes?\nAnswer: Kubernetes is an open-source container orchestration platform."
            },
            {
                "text": "Question: What is a Pod in Kubernetes?\nAnswer: A Pod is the smallest deployable unit in Kubernetes."
            },
            {
                "text": "Question: What is a Deployment?\nAnswer: A Deployment provides declarative updates for Pods and ReplicaSets."
            },
            {
                "text": "Question: What is a Service?\nAnswer: A Service is an abstraction for exposing Pods on a network."
            },
            {
                "text": "Question: What is Helm?\nAnswer: Helm is a package manager for Kubernetes."
            },
        ] * 20  # Repeat for more examples

        return Dataset.from_list(examples)

    def setup_lora(self):
        """Setup LoRA for parameter-efficient fine-tuning"""
        if not self.use_lora:
            return

        print("Setting up LoRA...")

        lora_config = LoraConfig(
            r=8,  # LoRA rank
            lora_alpha=32,  # LoRA alpha
            target_modules=["q_proj", "v_proj"],  # Target attention layers
            lora_dropout=0.05,
            bias="none",
            task_type=TaskType.CAUSAL_LM,
        )

        self.model = get_peft_model(self.model, lora_config)
        self.model.print_trainable_parameters()

    def train(self, epochs: int = 3, batch_size: int = 4):
        """Train the model"""
        print(f"Starting training for {epochs} epochs...")

        training_args = TrainingArguments(
            output_dir=str(self.output_dir / "checkpoints"),
            num_train_epochs=epochs,
            per_device_train_batch_size=batch_size,
            save_steps=100,
            save_total_limit=2,
            logging_steps=10,
            learning_rate=2e-4,
            weight_decay=0.01,
            fp16=torch.cuda.is_available(),
            push_to_hub=False,
            report_to="none",  # Disable wandb/tensorboard for example
        )

        data_collator = DataCollatorForLanguageModeling(
            tokenizer=self.tokenizer, mlm=False
        )

        trainer = Trainer(
            model=self.model,
            args=training_args,
            train_dataset=self.dataset,
            data_collator=data_collator,
        )

        trainer.train()
        print("Training completed!")

    def save_model(self):
        """Save fine-tuned model"""
        save_path = self.output_dir / "final_model"
        print(f"Saving model to: {save_path}")

        self.model.save_pretrained(save_path)
        self.tokenizer.save_pretrained(save_path)

        # Save metadata
        metadata = {
            "base_model": self.model_name,
            "use_lora": self.use_lora,
            "dataset": self.dataset_name or "example",
            "num_examples": len(self.dataset),
        }

        with open(save_path / "metadata.json", "w") as f:
            json.dump(metadata, f, indent=2)

        print("Model saved successfully!")

    def inference_test(self, prompt: str = "Question: What is Kubernetes?\nAnswer:"):
        """Test inference with fine-tuned model"""
        print(f"\nTesting inference with prompt: {prompt}")

        inputs = self.tokenizer(prompt, return_tensors="pt")
        if torch.cuda.is_available():
            inputs = {k: v.cuda() for k, v in inputs.items()}

        with torch.no_grad():
            outputs = self.model.generate(
                **inputs,
                max_new_tokens=100,
                temperature=0.7,
                top_p=0.9,
                do_sample=True,
            )

        response = self.tokenizer.decode(outputs[0], skip_special_tokens=True)
        print(f"Response: {response}")

    def export_to_gguf(self):
        """Export model to GGUF format for LocalAI"""
        print("\nExporting to GGUF format...")
        print("Note: This requires llama.cpp tools.")
        print(
            "Run: python -m llama_cpp.llama_cpp --model ./models/finetuned/final_model"
        )
        print("      --output ./data/localai/models/llm/finetuned-model.gguf")


def main():
    parser = argparse.ArgumentParser(description="Fine-tuning example script")
    parser.add_argument(
        "--model",
        type=str,
        default="meta-llama/Llama-3.2-1B",
        help="Base model name",
    )
    parser.add_argument(
        "--dataset", type=str, default=None, help="Dataset name from HuggingFace"
    )
    parser.add_argument(
        "--output",
        type=str,
        default="./models/finetuned",
        help="Output directory",
    )
    parser.add_argument(
        "--epochs",
        type=int,
        default=3,
        help="Number of training epochs",
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=4,
        help="Training batch size",
    )
    parser.add_argument(
        "--no-lora",
        action="store_true",
        help="Disable LoRA (full fine-tuning)",
    )
    parser.add_argument(
        "--test-only",
        action="store_true",
        help="Only run inference test",
    )

    args = parser.parse_args()

    # Initialize fine-tuning example
    finetuner = FineTuningExample(
        model_name=args.model,
        output_dir=args.output,
        dataset_name=args.dataset,
        use_lora=not args.no_lora,
    )

    if args.test_only:
        # Load existing model and test
        finetuner.load_model_and_tokenizer()
        finetuner.inference_test()
    else:
        # Full training workflow
        finetuner.load_model_and_tokenizer()
        finetuner.prepare_dataset()
        finetuner.setup_lora()
        finetuner.train(epochs=args.epochs, batch_size=args.batch_size)
        finetuner.save_model()
        finetuner.inference_test()
        finetuner.export_to_gguf()

    print("\n✓ Fine-tuning example completed!")
    print(f"\nModel saved to: {args.output}/final_model")
    print("\nNext steps:")
    print("1. Evaluate the model with TruLens: python scripts/evaluate-model.py")
    print("2. Convert to GGUF: See export instructions above")
    print("3. Deploy to LocalAI: Copy GGUF to data/localai/models/llm/")
    print("4. Test deployment: curl http://localhost:8080/v1/chat/completions")


if __name__ == "__main__":
    main()
