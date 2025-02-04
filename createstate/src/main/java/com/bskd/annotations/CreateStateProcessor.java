package com.bskd.annotations;

import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

import javax.annotation.processing.AbstractProcessor;
import javax.annotation.processing.Filer;
import javax.annotation.processing.ProcessingEnvironment;
import javax.annotation.processing.RoundEnvironment;
import javax.annotation.processing.SupportedAnnotationTypes;
import javax.annotation.processing.SupportedSourceVersion;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.Element;
import javax.lang.model.element.ElementKind;
import javax.lang.model.element.ExecutableElement;
import javax.lang.model.element.Modifier;
import javax.lang.model.element.TypeElement;
import javax.lang.model.util.Elements;
import javax.tools.Diagnostic;

import com.palantir.javapoet.*;

@SupportedAnnotationTypes({ "com.bskd.annotations.CreateState", "com.bskd.annotations.CreateStates" })
@SupportedSourceVersion(SourceVersion.RELEASE_17)
public class CreateStateProcessor extends AbstractProcessor {

    private Elements elementUtils;
    private Filer filer;

    @Override
    public synchronized void init(ProcessingEnvironment processingEnv) {
        super.init(processingEnv);
        elementUtils = processingEnv.getElementUtils();
        filer = processingEnv.getFiler();
    }

    @Override
    public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
        // Group methods by their enclosing class
        Map<TypeElement, Map<String, String>> classToStatesMap = new LinkedHashMap<>();

        for (Element element : roundEnv.getElementsAnnotatedWith(CreateStates.class)) {
            if (element.getKind() != ElementKind.METHOD) {
                continue;
            }

            ExecutableElement method = (ExecutableElement) element;
            TypeElement classElement = (TypeElement) method.getEnclosingElement();
            String methodName = method.getSimpleName().toString();

            classToStatesMap
                    .computeIfAbsent(classElement, k -> new LinkedHashMap<>())
                    .putAll(getStateMappings(method.getAnnotation(CreateStates.class), methodName));
        }

        // Handle @CreateState separately (single annotation case)
        for (Element element : roundEnv.getElementsAnnotatedWith(CreateState.class)) {
            if (element.getKind() != ElementKind.METHOD) {
                continue;
            }

            ExecutableElement method = (ExecutableElement) element;
            TypeElement classElement = (TypeElement) method.getEnclosingElement();
            String methodName = method.getSimpleName().toString();

            // Create a singleton mapping for a single @CreateState annotation
            Map<String, String> singleStateMapping = getStateMappings(method.getAnnotation(CreateState.class),
                    methodName);

            classToStatesMap
                    .computeIfAbsent(classElement, k -> new LinkedHashMap<>())
                    .putAll(singleStateMapping);
        }

        // Generate an enum for each class
        for (var entry : classToStatesMap.entrySet()) {
            try {
                genEnum(entry.getKey(), entry.getValue());
            } catch (IOException e) {
                processingEnv.getMessager().printMessage(Diagnostic.Kind.ERROR,
                        "Failed to generate enum: " + e.getMessage());
            }
        }

        return true;
    }

    private Map<String, String> getStateMappings(CreateStates annotation, String methodName) {
        Map<String, String> stateMap = new LinkedHashMap<>();
        if (annotation != null) {
            for (CreateState state : annotation.value()) {
                stateMap.put(state.value().toUpperCase(), methodName);
            }
        }
        return stateMap;
    }

    private Map<String, String> getStateMappings(CreateState annotation, String methodName) {
        Map<String, String> stateMap = new LinkedHashMap<>();
        if (annotation != null) {
            stateMap.put(annotation.value().toUpperCase(), methodName);
        }
        return stateMap;
    }

    private void genEnum(TypeElement classElement, Map<String, String> stateMap) throws IOException {
        String packageName = elementUtils.getPackageOf(classElement).getQualifiedName().toString();
        String className = classElement.getSimpleName().toString();
        String enumName = className + "States";

        // Define the Enum class
        TypeSpec.Builder enumBuilder = TypeSpec.enumBuilder(enumName)
                .addModifiers(Modifier.PUBLIC)
                .addMethod(MethodSpec.methodBuilder("execute")
                        .addModifiers(Modifier.PUBLIC, Modifier.ABSTRACT)
                        .addParameter(Object.class, "instance")
                        .build());

        for (Map.Entry<String, String> entry : stateMap.entrySet()) {
            String enumConstant = entry.getKey();
            String methodName = entry.getValue();

            TypeSpec subclass = TypeSpec.anonymousClassBuilder("")
                    .addMethod(MethodSpec.methodBuilder("execute")
                            .addModifiers(Modifier.PUBLIC)
                            .addAnnotation(Override.class)
                            .addParameter(Object.class, "instance")
                            .addStatement("(($T) instance).$L()", ClassName.get(classElement), methodName)
                            .build())
                    .build();

            enumBuilder.addEnumConstant(enumConstant, subclass);
        }

        JavaFile javaFile = JavaFile.builder(packageName, enumBuilder.build()).build();
        javaFile.writeTo(filer);
    }
}
