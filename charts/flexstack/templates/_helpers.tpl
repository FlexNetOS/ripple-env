{{/*
Expand the name of the chart.
*/}}
{{- define "flexstack.name" -}}
{{- default .Chart.Name .Values.nameOverride | trunc 63 | trimSuffix "-" }}
{{- end }}

{{/*
Create a default fully qualified app name.
*/}}
{{- define "flexstack.fullname" -}}
{{- if .Values.fullnameOverride }}
{{- .Values.fullnameOverride | trunc 63 | trimSuffix "-" }}
{{- else }}
{{- $name := default .Chart.Name .Values.nameOverride }}
{{- if contains $name .Release.Name }}
{{- .Release.Name | trunc 63 | trimSuffix "-" }}
{{- else }}
{{- printf "%s-%s" .Release.Name $name | trunc 63 | trimSuffix "-" }}
{{- end }}
{{- end }}
{{- end }}

{{/*
Create chart name and version as used by the chart label.
*/}}
{{- define "flexstack.chart" -}}
{{- printf "%s-%s" .Chart.Name .Chart.Version | replace "+" "_" | trunc 63 | trimSuffix "-" }}
{{- end }}

{{/*
Common labels
*/}}
{{- define "flexstack.labels" -}}
helm.sh/chart: {{ include "flexstack.chart" . }}
{{ include "flexstack.selectorLabels" . }}
{{- if .Chart.AppVersion }}
app.kubernetes.io/version: {{ .Chart.AppVersion | quote }}
{{- end }}
app.kubernetes.io/managed-by: {{ .Release.Service }}
app.kubernetes.io/part-of: flexstack
{{- end }}

{{/*
Selector labels
*/}}
{{- define "flexstack.selectorLabels" -}}
app.kubernetes.io/name: {{ include "flexstack.name" . }}
app.kubernetes.io/instance: {{ .Release.Name }}
{{- end }}

{{/*
Create the name of the service account to use
*/}}
{{- define "flexstack.serviceAccountName" -}}
{{- if .Values.serviceAccount.create }}
{{- default (include "flexstack.fullname" .) .Values.serviceAccount.name }}
{{- else }}
{{- default "default" .Values.serviceAccount.name }}
{{- end }}
{{- end }}

{{/*
Return the proper image name for LocalAI
*/}}
{{- define "flexstack.localai.image" -}}
{{- $repository := .Values.ai.localai.image.repository -}}
{{- $tag := .Values.ai.localai.image.tag | default "latest" -}}
{{- printf "%s:%s" $repository $tag -}}
{{- end }}

{{/*
Return the proper image name for AGiXT
*/}}
{{- define "flexstack.agixt.image" -}}
{{- $repository := .Values.ai.agixt.image.repository -}}
{{- $tag := .Values.ai.agixt.image.tag | default "main" -}}
{{- printf "%s:%s" $repository $tag -}}
{{- end }}

{{/*
Return PostgreSQL connection string
*/}}
{{- define "flexstack.postgresql.host" -}}
{{- if .Values.postgresql.enabled -}}
{{- printf "%s-postgresql" .Release.Name -}}
{{- else -}}
{{- .Values.externalDatabase.host -}}
{{- end -}}
{{- end }}

{{/*
Return Redis connection string
*/}}
{{- define "flexstack.redis.host" -}}
{{- if .Values.redis.enabled -}}
{{- printf "%s-redis-master" .Release.Name -}}
{{- else -}}
{{- .Values.externalRedis.host -}}
{{- end -}}
{{- end }}

{{/*
Return MinIO endpoint
*/}}
{{- define "flexstack.minio.endpoint" -}}
{{- if .Values.minio.enabled -}}
{{- printf "http://%s-minio:9000" .Release.Name -}}
{{- else -}}
{{- .Values.externalS3.endpoint -}}
{{- end -}}
{{- end }}

{{/*
Common environment variables for services
*/}}
{{- define "flexstack.commonEnv" -}}
- name: POSTGRES_HOST
  value: {{ include "flexstack.postgresql.host" . }}
- name: POSTGRES_PORT
  value: "5432"
- name: REDIS_HOST
  value: {{ include "flexstack.redis.host" . }}
- name: REDIS_PORT
  value: "6379"
- name: S3_ENDPOINT_URL
  value: {{ include "flexstack.minio.endpoint" . }}
{{- end }}
