# MOAI Infrastructure - Terraform

terraform {
  required_version = ">= 1.5.0"

  required_providers {
    google = {
      source  = "hashicorp/google"
      version = "~> 5.0"
    }
    aws = {
      source  = "hashicorp/aws"
      version = "~> 5.0"
    }
    kubernetes = {
      source  = "hashicorp/kubernetes"
      version = "~> 2.23"
    }
    helm = {
      source  = "hashicorp/helm"
      version = "~> 2.11"
    }
  }

  # Backend configuration - uncomment for production
  # backend "gcs" {
  #   bucket = "moai-terraform-state"
  #   prefix = "terraform/state"
  # }
}

# Variables
variable "project_id" {
  description = "GCP Project ID"
  type        = string
  default     = "moai-project"
}

variable "region" {
  description = "Deployment region"
  type        = string
  default     = "us-central1"
}

variable "cluster_name" {
  description = "GKE cluster name"
  type        = string
  default     = "moai-cluster"
}

variable "moai_image_tag" {
  description = "MOAI service image tag"
  type        = string
  default     = "latest"
}

# GCP Provider (for Cloud Robotics Core integration)
provider "google" {
  project = var.project_id
  region  = var.region
}

# GKE Cluster
resource "google_container_cluster" "moai" {
  name     = var.cluster_name
  location = var.region

  # Autopilot mode for managed nodes
  enable_autopilot = true

  # Network configuration
  network    = "default"
  subnetwork = "default"

  # Workload identity for secure service accounts
  workload_identity_config {
    workload_pool = "${var.project_id}.svc.id.goog"
  }
}

# Kubernetes provider using GKE cluster
provider "kubernetes" {
  host                   = "https://${google_container_cluster.moai.endpoint}"
  cluster_ca_certificate = base64decode(google_container_cluster.moai.master_auth[0].cluster_ca_certificate)
  token                  = data.google_client_config.default.access_token
}

data "google_client_config" "default" {}

# Helm provider
provider "helm" {
  kubernetes {
    host                   = "https://${google_container_cluster.moai.endpoint}"
    cluster_ca_certificate = base64decode(google_container_cluster.moai.master_auth[0].cluster_ca_certificate)
    token                  = data.google_client_config.default.access_token
  }
}

# MOAI Namespace
resource "kubernetes_namespace" "moai" {
  metadata {
    name = "moai"
    labels = {
      "app.kubernetes.io/name" = "moai"
    }
  }
}

# Deploy MOAI Service via Helm
resource "helm_release" "moai_service" {
  name       = "moai-service"
  namespace  = kubernetes_namespace.moai.metadata[0].name
  chart      = "${path.module}/../helm/moai-service"

  values = [
    yamlencode({
      image = {
        repository = "gcr.io/${var.project_id}/moai-service"
        tag        = var.moai_image_tag
      }
      replicaCount = 3
      resources = {
        limits = {
          cpu    = "4000m"
          memory = "8Gi"
        }
        requests = {
          cpu    = "1000m"
          memory = "2Gi"
        }
      }
      moai = {
        logLevel         = "INFO"
        sessionTtlSeconds = 3600
        workerPoolSize    = 8
        mtlsEnabled       = true
      }
    })
  ]

  depends_on = [kubernetes_namespace.moai]
}

# Outputs
output "cluster_endpoint" {
  value       = google_container_cluster.moai.endpoint
  description = "GKE cluster endpoint"
}

output "cluster_name" {
  value       = google_container_cluster.moai.name
  description = "GKE cluster name"
}

output "moai_namespace" {
  value       = kubernetes_namespace.moai.metadata[0].name
  description = "MOAI Kubernetes namespace"
}
