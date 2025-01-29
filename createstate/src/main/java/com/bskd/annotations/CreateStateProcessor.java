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
import javax.lang.model.type.TypeMirror;
import javax.lang.model.util.Elements;
import javax.tools.Diagnostic;

import com.palantir.javapoet.*;

@SupportedAnnotationTypes("com.bskd.annotations.CreateState")
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
        Map<String, String> methodMap = new LinkedHashMap<>();
        Map<String, String> classMap = new LinkedHashMap<>();

        for (Element element : roundEnv.getElementsAnnotatedWith(CreateState.class)) {
            if (element.getKind() != ElementKind.METHOD) {
                continue;
            }

            ExecutableElement method = (ExecutableElement) element;
            TypeElement classElement = (TypeElement) method.getEnclosingElement();

            String methodName = method.getSimpleName().toString();
            String className = classElement.getSimpleName().toString();
            String packageName = elementUtils.getPackageOf(classElement).getQualifiedName().toString();

            String enumConstant = element.getAnnotation(CreateState.class).value().toUpperCase();
            methodMap.put(enumConstant, methodName);
            classMap.put(enumConstant, className);

            try {
                genEnum(packageName, className, methodMap, classMap, method.getEnclosingElement());
            } catch (IOException e) {
                processingEnv.getMessager().printMessage(Diagnostic.Kind.ERROR,
                        "Failed to generate enum: " + e.getMessage() + ", error: " + e.getMessage());
            }
        }

        return true;
    }

    private void genEnum(String packageName, String className, Map<String, String> methodMap,
            Map<String, String> classMap, Element enclosingClass) throws IOException {
        String enumName = className + "StateEnum";

        // Define the Enum class
        TypeSpec.Builder enumBuilder = TypeSpec.enumBuilder(enumName)
                .addModifiers(Modifier.PUBLIC)
                .addMethod(MethodSpec.methodBuilder("execute")
                        .addModifiers(Modifier.PUBLIC)
                        .addParameter(Object.class, "instance").build());

        for (Map.Entry<String, String> entry : methodMap.entrySet()) {
            String enumConstant = entry.getKey();
            String methodName = entry.getValue();
            String containingClass = classMap.get(enumConstant);

            TypeSpec subclass = TypeSpec.anonymousClassBuilder("")
                    .addMethod(MethodSpec.methodBuilder("execute")
                            .addModifiers(Modifier.PUBLIC)
                            .addAnnotation(Override.class)
                            .addParameter(Object.class, "instance")
                            .addStatement("(($T) instance).$L()", ClassName.get(packageName, containingClass),
                                    methodName)
                            .build())
                    .build();

            enumBuilder.addEnumConstant(enumConstant, subclass);
        }

        JavaFile javaFile = JavaFile.builder(packageName, enumBuilder.build()).build();
        javaFile.writeTo(filer);
    }
}
