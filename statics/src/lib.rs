use proc_macro::TokenStream;
use quote::quote;
use syn::{
    bracketed,
    parse::Parse,
    parse_macro_input,
    Data,
    DeriveInput,
    Ident,
    LitInt,
    LitStr,
    Token,
};

fn _hash(data: &str) -> u32 {
    let mut prev_sum: u32 = 0;
    data.as_bytes()
        .iter()
        .map(|num| {
            prev_sum ^= *num as u32;
            prev_sum
        })
        .sum::<u32>()
}

#[proc_macro]
pub fn hash(item: TokenStream) -> TokenStream {
    let input = parse_macro_input!(item as LitStr).value();

    let hash = _hash(&input);

    println!("Hash {hash}");

    quote! {
        #hash
    }
    .into()
}

#[proc_macro]
pub fn string_to_bytes(item: TokenStream) -> TokenStream {
    let input = parse_macro_input!(item as LitStr).value();
    let bytes = input.as_bytes();

    let len = bytes.len().to_le_bytes();

    quote! {
        [#(#len,)*#(#bytes,)*]
    }
    .into()
}

struct IdentArray(Vec<Ident>);

impl Parse for IdentArray {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let content;
        bracketed!(content in input);
        let mut idents = Vec::new();
        while !content.is_empty() {
            idents.push(content.parse()?);
            let _: Token![,] = content.parse()?;
        }
        Ok(Self(idents))
    }
}

#[proc_macro]
pub fn numel(item: TokenStream) -> TokenStream {
    let input = parse_macro_input!(item as IdentArray).0.len();
    quote! {
        #input
    }
    .into()
}

struct PwmTableInput {
    timer_resolution: u8,
    pwm_resolution: u8,
}

impl Parse for PwmTableInput {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let timer: Ident = input.parse()?;
        if timer.to_string().to_lowercase() != "timer bits" {
            return syn::Result::Err(syn::Error::new_spanned(timer, "Expected \"timer bits\""));
        }
        let _: Token![:] = input.parse()?;
        let resolution: LitInt = input.parse()?;
        let timer_resolution = match u8::from_str_radix(resolution.clone().base10_digits(), 10) {
            Ok(val) => val,
            Err(_) => {
                return syn::Result::Err(
                    (syn::Error::new_spanned(resolution, "Expected a valid u8")),
                )
            }
        };

        let pwm: Ident = input.parse()?;
        if pwm.to_string().to_lowercase() != "pwm bits" {
            return syn::Result::Err(syn::Error::new_spanned(pwm, "Expected \"pwm bits\""));
        }
        let _: Token![:] = input.parse()?;
        let resolution: LitInt = input.parse()?;
        let pwm_resolution = match u8::from_str_radix(resolution.clone().base10_digits(), 10) {
            Ok(val) => val,
            Err(_) => {
                return syn::Result::Err(syn::Error::new_spanned(resolution, "Expected a valid u8"))
            }
        };

        Ok(Self {
            pwm_resolution,
            timer_resolution,
        })
    }
}




#[proc_macro]
/// Generates a LUT for all of possible pwm values.
pub fn generate_pwm_table(item: TokenStream) -> TokenStream {
    let input = parse_macro_input!(item as PwmTableInput);



    todo!()
}

#[proc_macro_derive(Iter)]
pub fn iter(item: TokenStream) -> TokenStream {
    let item = parse_macro_input!(item as DeriveInput);

    let variants = match item.data {
        Data::Enum(data) => data.variants,
        Data::Struct(_) | Data::Union(_) => panic!("input must be an enum"),
    };
    let variants: Vec<Ident> = variants.iter().map(|el| el.ident.clone()).collect();

    let name = item.ident.clone();
    assert!(
        item.generics.params.is_empty(),
        "Cannot be trivially implemented on enums with generics"
    );
    let n = variants.len();

    let new_ident = Ident::new(format!("Iterator{}", name).as_str(), name.span());

    // #item
    let ret = quote! {
        #[doc(hidden)]
        struct #new_ident {
            i:usize
        }
        impl #new_ident {
            const FIELDS:[#name;#n] = [#(#name::#variants,)*];
        }
        impl Iterator for #new_ident {
            type Item = #name;
            fn next(&mut self) -> Option<Self::Item> {
                self.i += 1;
                if self.i > #n {
                    return None;
                }
                return Some(Self::FIELDS[self.i - 1].clone())
            }
        }
        impl #name {
            const fn iter() -> #new_ident {
                #new_ident {
                    i: 0
                }
            }
        }
    };
    ret.into()
}

#[proc_macro_derive(Serializer)]
pub fn serialize(item: TokenStream) -> TokenStream {
    let item = parse_macro_input!(item as DeriveInput);
    let name = item.ident.clone();
    let mut field_names: Vec<Ident> = Vec::new();
    let mut field_types = Vec::new();

    let generics = item.generics.clone();
    let mut generic_names: Vec<Ident> = item
        .generics
        .lifetimes()
        .map(|el| el.lifetime.ident.clone())
        .collect();

    generic_names.extend(item.generics.type_params().map(|el| el.ident.clone()));
    generic_names.extend(item.generics.const_params().map(|el| el.ident.clone()));

    match &item.data {
        syn::Data::Struct(s) => {
            field_names.extend(
                s.fields
                    .clone()
                    .iter()
                    .map(|el| &el.ident)
                    .filter(|el| el.is_some())
                    .map(|el| unsafe { el.clone().unwrap_unchecked() }),
            );
            field_types.extend(s.fields.clone().iter().map(|el| el.ty.clone()));
        }
        t => {
            println!("FOUND TYPE {t:?}");
            panic!("Incompatible type.");
        }
    }

    let token = _hash(name.to_string().as_str());
    let intermediate: Vec<proc_macro2::TokenStream> = field_names
        .iter()
        .map(|el| {
            quote! {
                self.#el.into_bytes(writer);
            }
        })
        .collect();

    let intermediate_decode: Vec<proc_macro2::TokenStream> = field_names
        .iter()
        .zip(field_types.iter())
        .map(|(el, ty)| {
            quote! {
                let (n,intermediate_data) =match  #ty::from_bytes(ptr,data){
                    Ok(inner) => inner,
                    Err(_) => return Err(())
                };
                let #el = intermediate_data;
                ptr += n;
            }
        })
        .collect();

    let impl_line = match generic_names.len() {
        0 => quote! {#name},
        _ => quote! {#name <#(#generic_names,)*>},
    };

    // #item
    quote! {
        impl #generics #impl_line {
            const fn __size() -> usize {
                let mut size = core::mem::size_of::<#name>() + 4;
                const fn size_of<T: Serializable>() -> usize {
                    T::BUFFER_SIZE
                }
                #(
                    size += size_of::<#field_types>();
                )*
                size
            }
        }
        impl #generics Serializable for #impl_line {
                const BUFFER_SIZE: usize = {Self::__size()};
                type Error = ();
                /// Returns number of bytes used and buffer.
                fn into_bytes<'a,F:FnMut(u8)>(&'a self,mut writer:&mut F) {
                    let mut data = [0;Self::BUFFER_SIZE];

                    #token.to_le_bytes().iter().for_each(|el| writer(*el));
                    let mut ptr = 4;
                    #(#intermediate)*
                }
                fn from_bytes<'a>(ptr: usize, data: &'a mut [u8]) -> Result<(usize, Self), Self::Error>
                where
                    Self: Sized {
                    let old_ptr = ptr.clone();
                    if data.len() < (ptr +4) {
                        return Err(());
                    }
                    let data_repr = [data[ptr],data[ptr + 1],data[ptr+2],data[ptr + 3]];
                    if u32::from_le_bytes(data_repr) != #token {
                        return Err(());
                    }

                    let mut ptr = ptr + 2;
                    #(
                        #intermediate_decode
                    )*
                    let ptr = ptr-old_ptr;

                    Ok((ptr,Self {
                        #(
                            #field_names,
                        )*
                    }))
                }
        }
    }
    .into()
}
